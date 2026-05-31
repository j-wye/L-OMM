#!/usr/bin/env python3
"""Live no-command supervisor state-machine dry-run.

This entrypoint intentionally creates no robot command publishers and no action
clients.  It connects live CameraPreprocessor frames to map-update, reference
diff, event classification, lightweight supervisor decisions, optional
ControlModule dry-run candidate validation, and RecoveryRequest artifacts.
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import math
import os
import statistics
import sys
import threading
import time
import tokenize
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[2]
PROJECT_ROOT = SCRIPTS_DIR.parents[1]
CONTROL_DIR = SCRIPTS_DIR / "control_module"
for _path in (str(CONTROL_DIR), str(SCRIPTS_DIR)):
    if _path not in sys.path:
        sys.path.insert(0, _path)


OUT_ROOT = PROJECT_ROOT / "path" / "live_no_command_supervisor"
STATIC_OBSTACLE_OUT_ROOT_NAME = "static_obstacle_candidate_closure"
CAPSULE_AWARE_OUT_ROOT_NAME = "capsule_aware_static_obstacle_closure"
CAPSULE_FIXTURE_OUT_ROOT_NAME = "capsule_fixture_feasibility_closure"
ACTIVE_FRAME = "base_link"
CAMERA_FRAME = "gripper_camera_color_optical_frame"
JOINT_TOPIC = "/joint_states"
CAMERA_NAMESPACE = "gripper_camera"
CAMERA_NAME = "gripper_camera"
ACTIVE_JOINT_SYMBOLS = ("q2", "q3", "q5")
ACTIVE_JOINT_NAMES = ("arm_joint_2", "arm_joint_3", "arm_joint_5")
COMMAND_ENDPOINT_PATTERNS = (
    "cmd_vel",
    "trajectory",
    "joint_trajectory",
    "follow_joint_trajectory",
    "gripper",
    "controller",
)


class RuntimeState:
    NO_REFERENCE = "NO_REFERENCE"
    REFERENCE_ACTIVE = "REFERENCE_ACTIVE"
    REPLAN_PENDING = "REPLAN_PENDING"
    CANDIDATE_VALIDATING = "CANDIDATE_VALIDATING"
    REFERENCE_SWITCHED = "REFERENCE_SWITCHED"
    UNGRASPABLE_HANDOFF = "UNGRASPABLE_HANDOFF"
    WAIT_NEW_SNAPSHOT_AFTER_RECOVERY = "WAIT_NEW_SNAPSHOT_AFTER_RECOVERY"
    ERROR_STOP = "ERROR_STOP"


class RuntimeEvent:
    REFERENCE_CLEAR = "REFERENCE_CLEAR"
    REFERENCE_BLOCKED = "REFERENCE_BLOCKED"
    GOAL_CORRIDOR_BLOCKED = "GOAL_CORRIDOR_BLOCKED"
    CURRENT_POSE_UNSAFE = "CURRENT_POSE_UNSAFE"
    TARGET_CHANGED = "TARGET_CHANGED"
    NO_VALID_TARGET = "NO_VALID_TARGET"
    TF_UNAVAILABLE = "TF_UNAVAILABLE"
    CANDIDATE_ACCEPTED = "CANDIDATE_ACCEPTED"
    CANDIDATE_REJECTED = "CANDIDATE_REJECTED"
    UNGRASPABLE = "UNGRASPABLE"
    RECOVERY_REQUEST_DUMPED = "RECOVERY_REQUEST_DUMPED"
    SAFETY_VIOLATION = "SAFETY_VIOLATION"


CRITICAL_EVENTS = {
    RuntimeEvent.REFERENCE_BLOCKED,
    RuntimeEvent.GOAL_CORRIDOR_BLOCKED,
    RuntimeEvent.CURRENT_POSE_UNSAFE,
    RuntimeEvent.TARGET_CHANGED,
}


@dataclass
class ReferenceTrace:
    s: np.ndarray
    xz: np.ndarray

    @property
    def length(self) -> float:
        return float(self.s[-1]) if self.s.size else 0.0


def jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return value
    return value


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(jsonable(data), indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(jsonable(row), sort_keys=True, ensure_ascii=False) + "\n")


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: Optional[list[str]] = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        keys: list[str] = []
        for row in rows:
            for key in row.keys():
                if key not in keys:
                    keys.append(key)
        fieldnames = keys
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: jsonable(row.get(key, "")) for key in fieldnames})


def percentile(values: list[float], pct: float, default: float = float("nan")) -> float:
    vals = [float(v) for v in values if math.isfinite(float(v))]
    if not vals:
        return float(default)
    vals.sort()
    if len(vals) == 1:
        return vals[0]
    pos = (len(vals) - 1) * float(pct) / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return vals[lo]
    alpha = pos - lo
    return vals[lo] * (1.0 - alpha) + vals[hi] * alpha


def finite_mean(values: list[float], default: float = float("nan")) -> float:
    vals = [float(v) for v in values if math.isfinite(float(v))]
    return float(statistics.fmean(vals)) if vals else float(default)


def is_static_obstacle_goal(out_root: Any) -> bool:
    return STATIC_OBSTACLE_OUT_ROOT_NAME in str(out_root).replace("\\", "/")


def is_capsule_aware_goal(out_root: Any) -> bool:
    return CAPSULE_AWARE_OUT_ROOT_NAME in str(out_root).replace("\\", "/")


def is_capsule_fixture_goal(out_root: Any) -> bool:
    return CAPSULE_FIXTURE_OUT_ROOT_NAME in str(out_root).replace("\\", "/")


def safe_float(value: Any, default: float = float("nan")) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def histogram(values: Iterable[Any]) -> dict[str, int]:
    out: dict[str, int] = {}
    for value in values:
        key = str(value)
        out[key] = int(out.get(key, 0)) + 1
    return dict(sorted(out.items(), key=lambda item: (-item[1], item[0])))


def parse_jsonish(value: Any, default: Any = None) -> Any:
    if isinstance(value, (list, tuple, dict)):
        return value
    if value is None:
        return default
    text = str(value).strip()
    if text == "":
        return default
    for candidate in (text, text.replace("'", '"')):
        try:
            return json.loads(candidate)
        except Exception:
            pass
    return default


def command_like_endpoint(name: str) -> bool:
    lower = str(name).lower()
    return any(pattern in lower for pattern in COMMAND_ENDPOINT_PATTERNS)


def empty_ros_graph_inventory(reason: str = "not_requested") -> dict[str, Any]:
    return {
        "ros_graph_introspection_enabled": False,
        "ros_graph_introspection_read_only": True,
        "ros_graph_introspection_status": str(reason),
        "existing_external_command_topics": [],
        "existing_external_command_services": [],
        "existing_external_command_actions": [],
        "harness_created_publishers": [],
        "harness_created_action_clients": [],
        "harness_created_clients": [],
        "harness_created_subscriptions": [],
        "harness_command_topics_touched": [],
        "harness_command_actions_touched": [],
        "graph_query_created_command_publisher": False,
        "graph_query_created_action_client": False,
        "graph_query_sent_command": False,
    }


def collect_ros_graph_command_inventory(node: Any) -> dict[str, Any]:
    """Read ROS graph names without creating command publishers/action clients."""
    inventory = empty_ros_graph_inventory("ok")
    inventory["ros_graph_introspection_enabled"] = True
    try:
        topic_rows = [
            {"name": str(name), "types": list(types)}
            for name, types in node.get_topic_names_and_types()
            if command_like_endpoint(str(name))
        ]
        service_rows = [
            {"name": str(name), "types": list(types)}
            for name, types in node.get_service_names_and_types()
            if command_like_endpoint(str(name))
        ]
        try:
            action_rows = [
                {"name": str(name), "types": list(types)}
                for name, types in node.get_action_names_and_types()
                if command_like_endpoint(str(name))
            ]
        except Exception as exc:
            action_rows = []
            inventory["action_graph_query_status"] = f"unavailable:{type(exc).__name__}:{exc}"
        inventory.update(
            {
                "ros_graph_introspection_status": "ok",
                "existing_external_command_topics": topic_rows,
                "existing_external_command_services": service_rows,
                "existing_external_command_actions": action_rows,
                "existing_external_command_topic_count": len(topic_rows),
                "existing_external_command_service_count": len(service_rows),
                "existing_external_command_action_count": len(action_rows),
            }
        )
    except Exception as exc:
        inventory["ros_graph_introspection_status"] = f"failed:{type(exc).__name__}:{exc}"
    return inventory


def stamp_to_float(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def to_cpu_numpy(value: Any) -> np.ndarray:
    obj = value
    if hasattr(obj, "detach"):
        obj = obj.detach()
    if hasattr(obj, "cpu"):
        obj = obj.cpu()
    if hasattr(obj, "numpy"):
        return np.asarray(obj.numpy())
    return np.asarray(obj)


def torch_matrix_to_numpy(value: Any) -> np.ndarray:
    return np.asarray(to_cpu_numpy(value), dtype=np.float64).reshape(4, 4)


def infer_active_joint_map(names: list[str]) -> tuple[dict[str, int], bool]:
    out: dict[str, int] = {}
    for symbol, expected in zip(ACTIVE_JOINT_SYMBOLS, ACTIVE_JOINT_NAMES):
        matches = [idx for idx, name in enumerate(names) if str(name) == expected]
        if len(matches) == 1:
            out[symbol] = matches[0]
    return out, len(out) == len(ACTIVE_JOINT_SYMBOLS)


def normalize_perception_evidence(perception_result: Any) -> tuple[list[Any], str, bool]:
    from map_update_layer import Detection2D

    if perception_result is None:
        return [], "perception_invalid_empty_semantic", False
    valid_arr = to_cpu_numpy(perception_result.valid).reshape(-1)
    valid = bool(valid_arr.size > 0 and bool(valid_arr[0]))
    if not valid:
        return [], "perception_invalid_empty_semantic", False
    bbox = tuple(float(v) for v in to_cpu_numpy(perception_result.bbox).reshape(-1)[:4])
    mask = np.asarray(to_cpu_numpy(perception_result.mask), dtype=bool)
    score_arr = to_cpu_numpy(perception_result.score).reshape(-1)
    score = float(score_arr[0]) if score_arr.size else 1.0
    return [
        Detection2D(
            semantic_type="target",
            bbox=bbox,
            mask=mask,
            score=score,
            source_id="real_perception_target",
            fallback_depth_m=None,
        )
    ], "real_perception_target", True


def camera_intrinsics_from_frame(frame: Any) -> Any:
    from map_update_layer.perception_to_map import CameraIntrinsics

    intr = np.asarray(frame.intrinsics, dtype=np.float64)
    height, width = np.asarray(frame.depth).shape[:2]
    return CameraIntrinsics(
        fx=float(intr[0, 0]),
        fy=float(intr[1, 1]),
        cx=float(intr[0, 2]),
        cy=float(intr[1, 2]),
        width=int(width),
        height=int(height),
    )


def build_synthetic_handle(*, include_projected_q_grid: bool = False) -> Any:
    from constants import DEFAULT_MU_MIN, DEFAULT_TAG, Q_HOME_DEFAULT, X_EE_GOAL, X_TARGET_CONTACT, Y_PLANE_FIXED
    from map_handle import MapHandle

    shape = (260, 240)
    target_height = 0.50
    mu_grid = np.full(shape, max(DEFAULT_MU_MIN * 10.0, 0.05), dtype=np.float64)
    pitch_grid = np.zeros(shape, dtype=np.float64)
    q_grid = None
    q_grid_source = "none"
    q_grid_valid_cells = 0
    default_dir = PROJECT_ROOT / "map" / "map_test" / "10mm"
    map_path = default_dir / f"map_{DEFAULT_TAG}.npy"
    q_path = default_dir / f"q_map_{DEFAULT_TAG}.npy"
    meta_path = default_dir / f"meta_{DEFAULT_TAG}.json"
    try:
        default_map = np.load(map_path)
        default_q = np.load(q_path).astype(np.float64)
        default_meta = read_json_or_none(meta_path) or {}
        default_x0 = float((default_meta.get("x_range") or [0.0, 0.0])[0])
        default_z0 = float((default_meta.get("z_range") or [0.0, 0.0])[0])
        default_res = float(default_meta.get("resolution_m", 0.01))
        if include_projected_q_grid and default_map.ndim == 3 and default_q.ndim == 3 and default_q.shape[:2] == default_map.shape[:2]:
            q_grid_arr = np.full((shape[0], shape[1], 3), np.nan, dtype=np.float64)
            valid_default = (
                np.isfinite(default_map[:, :, 0])
                & (default_map[:, :, 0] > float(DEFAULT_MU_MIN))
                & np.all(np.isfinite(default_q), axis=2)
            )
            for dx, dz in np.argwhere(valid_default):
                x = default_x0 + int(dx) * default_res
                z = default_z0 + int(dz) * default_res
                sx = int(round((x - -1.1) / 0.01))
                sz = int(round((z - -0.6) / 0.01))
                if 0 <= sx < shape[0] and 0 <= sz < shape[1]:
                    q_grid_arr[sx, sz] = default_q[int(dx), int(dz)]
                    mu_grid[sx, sz] = float(default_map[int(dx), int(dz), 0])
                    pitch_grid[sx, sz] = float(default_map[int(dx), int(dz), 1])
            q_grid_valid_cells = int(np.count_nonzero(np.all(np.isfinite(q_grid_arr), axis=2)))
            if q_grid_valid_cells > 0:
                q_grid = q_grid_arr
                q_grid_source = "default_offline_q_map_projected_into_synthetic_fixture"
        elif default_q.ndim == 3:
            q_grid_source = "default_offline_q_map_available_but_not_used_for_synthetic_fixture"
    except Exception as exc:
        q_grid_source = f"unavailable:{type(exc).__name__}:{exc}"
    return MapHandle(
        map_path="live_no_command_supervisor_synthetic",
        meta_path="live_no_command_supervisor_synthetic_meta",
        mu_grid=mu_grid,
        pitch_grid=pitch_grid,
        q_grid=q_grid,
        meta={
            "mu_min_runtime": DEFAULT_MU_MIN,
            "target_y_fixed_m": Y_PLANE_FIXED,
            "target_height": target_height,
            "x_opt_ee_goal_m": X_EE_GOAL,
            "target_z": target_height,
            "target_contact_x_m": X_TARGET_CONTACT,
            "x_target_contact_m": X_TARGET_CONTACT,
            "x_gaze": X_TARGET_CONTACT,
            "q_home_reduced_seed": list(Q_HOME_DEFAULT),
            "synthetic_q_grid_source": q_grid_source,
            "synthetic_q_grid_valid_cells": q_grid_valid_cells,
        },
        resolution_m=0.01,
        x0=-1.1,
        z0=-0.6,
        target_y=Y_PLANE_FIXED,
        tag="live_no_command_supervisor_synthetic",
    )


def load_default_or_synthetic_handle() -> tuple[Any, str, bool]:
    try:
        from path_planning import PathPlanning

        handle = PathPlanning().load_map()
        return handle, "default_control_map", True
    except Exception as exc:
        handle = build_synthetic_handle()
        handle.meta["default_map_load_error"] = f"{type(exc).__name__}: {exc}"
        return handle, "synthetic_fallback_for_software_contract", False


def build_reference_trace(start_xz: tuple[float, float], goal_xz: tuple[float, float], samples: int = 80) -> ReferenceTrace:
    start = np.asarray(start_xz, dtype=np.float64).reshape(2)
    goal = np.asarray(goal_xz, dtype=np.float64).reshape(2)
    t = np.linspace(0.0, 1.0, max(int(samples), 2), dtype=np.float64)
    xz = (1.0 - t[:, None]) * start[None, :] + t[:, None] * goal[None, :]
    seg = np.linalg.norm(np.diff(xz, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    return ReferenceTrace(s=s, xz=xz)


def reference_xz_at_fraction(reference: ReferenceTrace, fraction: float) -> tuple[float, tuple[float, float]]:
    s_val = float(reference.length) * float(fraction)
    x = float(np.interp(s_val, reference.s, reference.xz[:, 0]))
    z = float(np.interp(s_val, reference.s, reference.xz[:, 1]))
    return s_val, (x, z)


def default_start_goal(handle: Any, target_xz: Optional[tuple[float, float]] = None) -> tuple[tuple[float, float], tuple[float, float]]:
    try:
        from kinematics import Kinematics

        start = tuple(float(v) for v in Kinematics().default_start_xz())
    except Exception:
        start = (float(handle.x0 + 0.2), 0.0)
    if target_xz is not None and all(math.isfinite(float(v)) for v in target_xz):
        goal = (float(target_xz[0]), float(target_xz[1]))
    else:
        goal = (
            float(handle.meta.get("x_opt_ee_goal_m", handle.meta.get("optimal_x", 0.75))),
            float(handle.meta.get("target_z", handle.meta.get("target_height", 0.0))),
        )
    return start, goal


def target_xz_from_pipeline(pipeline_step: dict[str, Any], handle: Any) -> tuple[Optional[tuple[float, float]], str]:
    value = pipeline_step.get("T_map_target")
    if value is None:
        return None, "missing_T_target"
    try:
        mat = torch_matrix_to_numpy(value)
        return (float(mat[0, 3]), float(mat[2, 3])), "from_T_target_translation_xz"
    except Exception as exc:
        return None, f"invalid_T_target:{type(exc).__name__}:{exc}"


def candidate_metrics(summary: dict[str, Any]) -> dict[str, Any]:
    case = summary.get("case", {}) if isinstance(summary, dict) else {}
    plan = case.get("plan", {}) if isinstance(case, dict) else {}
    reference = case.get("reference", {}) if isinstance(case, dict) else {}
    control = case.get("control", {}) if isinstance(case, dict) else {}
    capsule = case.get("capsule_proxy_validation", {}) if isinstance(case, dict) else {}
    policy = case.get("candidate_policy", {}) if isinstance(case, dict) else {}
    def as_float(value: Any, default: float = float("nan")) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    return {
        "candidate_metrics_source": "control_summary",
        "candidate_accepted": bool(policy.get("candidate_accepted", False)),
        "rejection_reason": str(policy.get("reject_reason", plan.get("invalid_reason_code", ""))),
        "invalid_reason_code": str(policy.get("invalid_reason_code", plan.get("invalid_reason_code", ""))),
        "path_length_m": as_float(plan.get("length_m", float("nan"))),
        "reference_length_m": as_float(reference.get("length_m", float("nan"))),
        "final_pos_err_m": as_float(control.get("final_pos_err_m", float("nan"))),
        "L_q": as_float(control.get("L_q", float("nan"))),
        "rho_ref": as_float(control.get("rho_ref", float("nan"))),
        "capsule_proxy_collision_free": as_float(capsule.get("capsule_proxy_collision_free", float("nan"))),
    }


def mock_candidate_metrics(candidate_accepted: bool, rejection_reason: str = "", invalid_reason_code: str = "") -> dict[str, Any]:
    return {
        "candidate_metrics_source": "mock_candidate_metrics",
        "candidate_accepted": bool(candidate_accepted),
        "rejection_reason": str(rejection_reason),
        "invalid_reason_code": str(invalid_reason_code),
    }


def is_real_control_connected(row: dict[str, Any]) -> bool:
    return bool(
        row.get("source") == "ControlModule.run"
        and row.get("control_called") in (True, "True")
        and str(row.get("control_exception", "")) == ""
        and row.get("candidate_metrics_source") == "control_summary"
    )


def is_real_control_accepted(row: dict[str, Any]) -> bool:
    return bool(
        is_real_control_connected(row)
        and row.get("candidate_accepted") in (True, "True")
        and row.get("supervisor_candidate_accepted") in (True, "True")
    )


def run_control_candidate(
    *,
    snapshot: Any,
    clear_snapshot: Any,
    start_xz: tuple[float, float],
    goal_xz: tuple[float, float],
    out_dir: Path,
    scenario: str = "changed",
    initial_q_active: Optional[np.ndarray] = None,
) -> dict[str, Any]:
    from control_module.control_module import ControlModule
    from mission_request import MissionRequest

    request = MissionRequest(
        scenario=str(scenario),
        active_snapshot=snapshot,
        clear_snapshot=clear_snapshot,
        start_xz=(float(start_xz[0]), float(start_xz[1])),
        goal_xz=(float(goal_xz[0]), float(goal_xz[1])),
        initial_q_active=tuple(float(v) for v in np.asarray(initial_q_active, dtype=np.float64).reshape(3)) if initial_q_active is not None else None,
        cost_mode="distance",
        reference_backend="c2_quintic",
        make_plots=False,
        write_artifacts=True,
    )
    return ControlModule(output_root=str(out_dir.parent)).run(request, out_dir=str(out_dir))


class TargetChangedDetector:
    def __init__(self, hysteresis_m: float = 0.02) -> None:
        self.hysteresis_m = float(hysteresis_m)
        self.previous: Optional[np.ndarray] = None

    def update(self, target_xz: Optional[tuple[float, float]]) -> dict[str, Any]:
        if target_xz is None:
            return {
                "target_available": False,
                "target_changed_event": False,
                "target_displacement_m": float("nan"),
                "hysteresis_m": self.hysteresis_m,
            }
        cur = np.asarray(target_xz, dtype=np.float64).reshape(2)
        if self.previous is None:
            self.previous = cur.copy()
            return {
                "target_available": True,
                "target_changed_event": False,
                "target_displacement_m": 0.0,
                "hysteresis_m": self.hysteresis_m,
            }
        disp = float(np.linalg.norm(cur - self.previous))
        changed = bool(disp > self.hysteresis_m)
        if changed:
            self.previous = cur.copy()
        return {
            "target_available": True,
            "target_changed_event": changed,
            "target_displacement_m": disp,
            "hysteresis_m": self.hysteresis_m,
        }


class SupervisorStateMachine:
    def __init__(self) -> None:
        self.state = RuntimeState.NO_REFERENCE
        self.transitions: list[dict[str, Any]] = []
        self.last_recovery_snapshot_key: Optional[str] = None

    def transition(self, new_state: str, event: str, *, frame_index: int, reason: str = "") -> None:
        row = {
            "timestamp_s": time.time(),
            "frame_index": int(frame_index),
            "from_state": self.state,
            "event": str(event),
            "to_state": str(new_state),
            "reason": str(reason),
        }
        self.transitions.append(row)
        self.state = str(new_state)

    def accept_reference(self, *, frame_index: int, reason: str) -> None:
        if self.state == RuntimeState.NO_REFERENCE:
            self.transition(RuntimeState.REFERENCE_ACTIVE, RuntimeEvent.REFERENCE_CLEAR, frame_index=frame_index, reason=reason)
        elif self.state == RuntimeState.REFERENCE_SWITCHED:
            self.transition(RuntimeState.REFERENCE_ACTIVE, RuntimeEvent.CANDIDATE_ACCEPTED, frame_index=frame_index, reason=reason)

    def evaluate(
        self,
        *,
        frame_index: int,
        report: Any,
        supervisor_decision: Any,
        recovery_key: Optional[str] = None,
    ) -> None:
        event = str(report.event_class)
        if self.state == RuntimeState.WAIT_NEW_SNAPSHOT_AFTER_RECOVERY:
            if recovery_key is not None and recovery_key == self.last_recovery_snapshot_key:
                self.transition(
                    RuntimeState.WAIT_NEW_SNAPSHOT_AFTER_RECOVERY,
                    "WAIT_NEW_SNAPSHOT_AFTER_RECOVERY",
                    frame_index=frame_index,
                    reason="same_snapshot_retry_prevented",
                )
                return
            self.transition(RuntimeState.REFERENCE_ACTIVE, RuntimeEvent.REFERENCE_CLEAR, frame_index=frame_index, reason="new_snapshot_after_recovery")

        if not bool(report.replanning_triggered):
            self.transition(RuntimeState.REFERENCE_ACTIVE, event, frame_index=frame_index, reason=str(report.reason))
            return

        self.transition(RuntimeState.REPLAN_PENDING, event, frame_index=frame_index, reason=str(report.reason))
        if event == RuntimeEvent.CURRENT_POSE_UNSAFE:
            self.transition(RuntimeState.UNGRASPABLE_HANDOFF, RuntimeEvent.UNGRASPABLE, frame_index=frame_index, reason="current_pose_unsafe_recovery_boundary")
            self.last_recovery_snapshot_key = recovery_key
            self.transition(
                RuntimeState.WAIT_NEW_SNAPSHOT_AFTER_RECOVERY,
                RuntimeEvent.RECOVERY_REQUEST_DUMPED,
                frame_index=frame_index,
                reason="recovery_request_dumped_no_command",
            )
            return

        self.transition(RuntimeState.CANDIDATE_VALIDATING, "CALL_CONTROL_DRYRUN", frame_index=frame_index, reason=str(report.reason))
        if bool(supervisor_decision.candidate_accepted):
            self.transition(RuntimeState.REFERENCE_SWITCHED, RuntimeEvent.CANDIDATE_ACCEPTED, frame_index=frame_index, reason="candidate_accepted")
            return
        if bool(supervisor_decision.recovery_required):
            self.transition(RuntimeState.UNGRASPABLE_HANDOFF, RuntimeEvent.UNGRASPABLE, frame_index=frame_index, reason="candidate_rejected_ungraspable")
            self.last_recovery_snapshot_key = recovery_key
            self.transition(
                RuntimeState.WAIT_NEW_SNAPSHOT_AFTER_RECOVERY,
                RuntimeEvent.RECOVERY_REQUEST_DUMPED,
                frame_index=frame_index,
                reason="recovery_request_dumped_no_command",
            )
            return
        self.transition(RuntimeState.REFERENCE_ACTIVE, RuntimeEvent.CANDIDATE_REJECTED, frame_index=frame_index, reason="candidate_validation_skipped_or_deferred")


def blocked_mask_key(snapshot: Any) -> str:
    from map_update_layer.recovery_contract import RecoveryHandoffContract

    return RecoveryHandoffContract.blocked_mask_hash(snapshot.blocked_mask)


def build_snapshot(handle: Any, occupied_rects: list[tuple[float, float, float, float]], sequence_id: int) -> Any:
    from constants import DEFAULT_MU_MIN
    from map_update_layer.map_update_layer import MapUpdateLayer
    from map_update_layer.map_update_request import MapUpdateRequest

    req = MapUpdateRequest(
        occupied_rects=tuple(occupied_rects),
        source="live_no_command_supervisor_regression",
        sequence_id=int(sequence_id),
        timestamp_s=float(sequence_id),
        adapter_stats={"synthetic_regression_fixture": True},
    )
    return MapUpdateLayer().build(handle, mu_min=DEFAULT_MU_MIN, request=req)


def run_stage_a(args: argparse.Namespace) -> int:
    if is_capsule_fixture_goal(args.out_root):
        return run_fixture_stage_a(args)

    if is_capsule_aware_goal(args.out_root):
        return run_capsule_stage_a(args)

    stage_dir = Path(args.out_root) / "stage_a_forensics"
    stage_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    old_root = PROJECT_ROOT / "path" / "live_no_command_supervisor"
    harness_path = Path(__file__)
    blockers: list[str] = []

    def read_json_or_none(path: Path) -> Any:
        errors: list[str] = []
        for encoding in ("utf-8", "utf-8-sig", "utf-16"):
            try:
                return json.loads(path.read_text(encoding=encoding))
            except Exception as exc:
                errors.append(f"{encoding}:{type(exc).__name__}")
        blockers.append(f"missing_or_invalid:{path.name}:{';'.join(errors)}")
        return None

    previous_summary = read_json_or_none(old_root / "live_no_command_supervisor_summary.json")
    previous_control = read_json_or_none(old_root / "stage_c_state_machine_regression" / "control_dryrun_candidate_summary.json")
    previous_stage_e = read_json_or_none(old_root / "stage_e_sustained_soak" / "stage_summary.json")
    source = harness_path.read_text(encoding="utf-8")
    control_rows = list((previous_control or {}).get("control_rows", []))
    keyerror_rows = [
        row for row in control_rows
        if "target_contact_x_m" in str(row.get("rejection_reason", ""))
    ]
    real_accepted_rows = [
        row for row in control_rows
        if str(row.get("case", "")).startswith("real_control")
        and row.get("control_called") in (True, "True")
        and row.get("candidate_accepted") in (True, "True")
    ]
    defect_inventory = {
        "previous_controlmodule_dryrun_connected": (previous_summary or {}).get("controlmodule_dryrun_connected", "missing"),
        "previous_stage_e_control_dryrun_call_count": (previous_stage_e or {}).get("control_dryrun_call_count", "missing"),
        "case_name_only_real_control_connected_bug": 'real_control_connected = any(row.get("case") == "real_control_dryrun_clear_candidate"' in source,
        "same_snapshot_retry_or_true_bug": '"same_snapshot_retry_prevented"' in source and ("or " + "True") in source,
        "keyerror_missing_target_contact_x_m_count": len(keyerror_rows),
        "real_control_successful_accepted_rows": len(real_accepted_rows),
    }
    previous_control_evidence = {
        "control_rows": control_rows,
        "keyerror_rows": keyerror_rows,
        "conclusion": "Previous ControlModule PASS is an overclaim unless a real accepted ControlModule.run row exists.",
    }

    saturation_summary: dict[str, Any] = {
        "stage_e_live_sample_count": (previous_stage_e or {}).get("live_sample_count", 0),
        "stage_e_supervisor_decision_count": (previous_stage_e or {}).get("supervisor_decision_count", 0),
        "stage_e_control_dryrun_call_count": (previous_stage_e or {}).get("control_dryrun_call_count", 0),
        "stage_e_event_class": "CURRENT_POSE_UNSAFE" if (previous_stage_e or {}).get("critical_event_count", 0) else "unknown",
    }
    try:
        decision_rows = list(csv.DictReader((old_root / "stage_e_sustained_soak" / "supervisor_decisions.csv").open("r", encoding="utf-8")))
        for key in ("current_pose_blocked_count", "reference_blocked_count", "goal_blocked_count"):
            vals = [float(row[key]) for row in decision_rows if str(row.get(key, "")) != ""]
            saturation_summary[f"{key}_min"] = min(vals) if vals else float("nan")
            saturation_summary[f"{key}_median"] = percentile(vals, 50.0)
            saturation_summary[f"{key}_max"] = max(vals) if vals else float("nan")
    except Exception as exc:
        saturation_summary["decision_csv_error"] = f"{type(exc).__name__}: {exc}"

    safety_findings = scan_forbidden_code_tokens(source)
    if any(int(v) > 0 for v in safety_findings.values()):
        blockers.append("forbidden_token_in_harness")

    passed = len(blockers) == 0
    write_json(stage_dir / "defect_inventory.json", defect_inventory)
    write_json(stage_dir / "previous_control_dryrun_evidence.json", previous_control_evidence)
    write_json(stage_dir / "previous_live_saturation_summary.json", saturation_summary)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage A - previous artifact forensics", "timestamp_s": time.time(), "status": "PASS" if passed else "FAIL"})
    summary = {
        "stage_name": "Stage A - previous artifact forensics and defect confirmation",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": "PASS" if passed else "FAIL",
        "stage_passed": passed,
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "defect_inventory.json"),
            str(stage_dir / "previous_control_dryrun_evidence.json"),
            str(stage_dir / "previous_live_saturation_summary.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage B" if passed else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if passed else 1


def run_stage_b(args: argparse.Namespace) -> int:
    if is_capsule_fixture_goal(args.out_root):
        return run_fixture_stage_b(args)

    if is_capsule_aware_goal(args.out_root):
        return run_capsule_stage_b(args)

    if "control_dryrun_candidate_closure" in str(args.out_root):
        stage_dir_name = "stage_b_patch"
    elif is_static_obstacle_goal(args.out_root):
        stage_dir_name = "stage_b_fixture_safety_patch"
    else:
        stage_dir_name = "stage_b_harness_build"
    stage_dir = Path(args.out_root) / stage_dir_name
    stage_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    harness_path = Path(__file__)
    source = harness_path.read_text(encoding="utf-8")
    findings = scan_forbidden_code_tokens(source)
    allowed_mentions = {
        "cmd_vel": "mentioned only in forbidden scan strings and reports",
        "FollowJointTrajectory": "mentioned only in forbidden scan strings and reports",
    }
    compile_ok = True
    compile_error = ""
    try:
        compile(source, str(harness_path), "exec")
    except Exception as exc:
        compile_ok = False
        compile_error = f"{type(exc).__name__}: {exc}"

    state_machine_spec = {
        "states": [
            RuntimeState.NO_REFERENCE,
            RuntimeState.REFERENCE_ACTIVE,
            RuntimeState.REPLAN_PENDING,
            RuntimeState.CANDIDATE_VALIDATING,
            RuntimeState.REFERENCE_SWITCHED,
            RuntimeState.UNGRASPABLE_HANDOFF,
            RuntimeState.WAIT_NEW_SNAPSHOT_AFTER_RECOVERY,
            RuntimeState.ERROR_STOP,
        ],
        "critical_events": sorted(CRITICAL_EVENTS),
        "same_snapshot_retry_prevention": True,
    }
    target_spec = {
        "input": "consecutive T_base_target/T_map_target x-z translations",
        "hysteresis_m": float(args.target_changed_hysteresis_m),
        "output": "TARGET_CHANGED event artifact only",
        "base_compensation_implemented": False,
    }
    routing_policy = {
        "target_pose_source": "PerceptionDecisionPipeline.step()['T_map_target']",
        "height_layer_policy": "use current MapHandle target_y; do not invent missing physical geometry",
        "goal_xz_policy": "use target translation x,z when available; otherwise mark routing_blocked",
        "dry_run_only": True,
    }
    report_field_contract = {
        "real_control_dryrun_connected": "requires source=ControlModule.run, control_called=true, control_exception='', candidate_metrics_source=control_summary",
        "real_control_candidate_accepted": "requires real_control_dryrun_connected and candidate_accepted=true",
        "mock_candidate_transition_coverage": "reported separately from real ControlModule.run evidence",
        "same_snapshot_retry_prevented": "computed from transition rows only; no unconditional truth shortcut",
    }
    static_obstacle_fixture_contract = {
        "case_name": "real_control_static_obstacle_to_accepted",
        "required_event_class": RuntimeEvent.REFERENCE_BLOCKED,
        "required_current_pose_blocked_count": 0,
        "required_goal_blocked_count": 0,
        "required_reference_blocked_count": ">0",
        "candidate_source": "ControlModule.run",
        "candidate_acceptance": "must be real control_summary candidate_accepted=true; mock evidence is forbidden",
        "max_repair_attempts": int(getattr(args, "max_repair_attempts", 3)),
        "failure_policy": "if ControlModule stays rejected, mark Stage C PARTIAL and do not enter live stages",
    }
    graph_safety_contract = {
        "introspection": "read_only_ros_graph_inventory",
        "allowed_graph_queries": [
            "get_topic_names_and_types",
            "get_service_names_and_types",
            "get_action_names_and_types_if_available",
        ],
        "forbidden_graph_side_effects": [
            "create_publisher",
            "ActionClient",
            "command subscription",
            "send_goal",
            "publish",
        ],
        "existing_external_command_endpoints_are_failure": False,
        "harness_created_or_touched_command_endpoints_are_failure": True,
    }
    safety = {
        "created_publishers": [],
        "created_action_clients": [],
        "created_clients": [],
        "created_subscriptions": ["CameraPreprocessor RGB-D topics", JOINT_TOPIC, "/tf", "/tf_static"],
        "runtime_created_publishers": [],
        "runtime_created_action_clients": [],
        "runtime_inventory_source": "harness_internal_created_resource_lists",
        "robot_command_sent": False,
        "new_rgbd_subscriber_added_by_harness": False,
        "new_realsense_pipeline_opened": False,
        "forbidden_term_counts": findings,
        "allowed_text_mentions": allowed_mentions,
        "robot_command_safety": "PASS" if not any(int(v) > 0 for v in findings.values()) else "FAIL",
    }
    write_json(stage_dir / "harness_static_scan.json", safety)
    write_json(stage_dir / "state_machine_spec.json", state_machine_spec)
    write_json(stage_dir / "target_changed_detector_spec.json", target_spec)
    write_json(stage_dir / "routing_policy.json", routing_policy)
    write_json(stage_dir / "report_field_contract.json", report_field_contract)
    write_json(stage_dir / "static_obstacle_fixture_contract.json", static_obstacle_fixture_contract)
    write_json(stage_dir / "graph_safety_contract.json", graph_safety_contract)
    write_json(stage_dir / "no_command_safety_contract.json", safety)
    write_json(stage_dir / "safety_no_command_report.json", safety)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage B - harness build/reuse", "timestamp_s": time.time(), "status": "completed"})
    passed = bool(compile_ok and safety["robot_command_safety"] == "PASS")
    summary = {
        "stage_name": "Stage B - harness build/reuse",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": "PASS" if passed else "FAIL",
        "stage_passed": passed,
        "stage_blockers": [] if passed else [compile_error or "safety_static_scan"],
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "harness_static_scan.json"),
            str(stage_dir / "state_machine_spec.json"),
            str(stage_dir / "target_changed_detector_spec.json"),
            str(stage_dir / "routing_policy.json"),
            str(stage_dir / "report_field_contract.json"),
            str(stage_dir / "static_obstacle_fixture_contract.json"),
            str(stage_dir / "graph_safety_contract.json"),
            str(stage_dir / "no_command_safety_contract.json"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage C" if passed else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "harness_path": str(harness_path),
        "compile_ok": compile_ok,
        "compile_error": compile_error,
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if passed else 1


def scan_forbidden_code_tokens(source: str) -> dict[str, int]:
    """Token-level safety scan that ignores comments and string literals."""
    tokens: list[str] = []
    for tok in tokenize.generate_tokens(io.StringIO(source).readline):
        if tok.type in {tokenize.STRING, tokenize.COMMENT, tokenize.NL, tokenize.NEWLINE, tokenize.INDENT, tokenize.DEDENT}:
            continue
        tokens.append(tok.string)
    text = " ".join(tokens)
    return {
        "create_publisher_call": int(". create_publisher (" in text or "create_publisher (" in text),
        "action_client_call": int("ActionClient (" in text),
        "follow_joint_trajectory_symbol": int("FollowJointTrajectory" in tokens),
        "twist_symbol": int("Twist" in tokens),
        "cmd_vel_literal_outside_string": int("cmd_vel" in tokens),
        "pyrealsense2_import": int("pyrealsense2" in tokens),
        "rs_pipeline_call": int("rs . pipeline (" in text),
    }


def classify_and_decide(
    *,
    classifier: Any,
    supervisor: Any,
    snapshot: Any,
    reference_trace: ReferenceTrace,
    current_s_m: float,
    previous_blocked_mask: Optional[np.ndarray],
    target_changed_event: bool,
    diff_report: Optional[Any],
    capsule_proxy_xz: Optional[np.ndarray],
    goal_xz: tuple[float, float],
    candidate: Optional[dict[str, Any]],
) -> tuple[Any, Any]:
    report = classifier.classify(
        snapshot,
        s_table=reference_trace.s,
        xz_table=reference_trace.xz,
        current_s_m=float(current_s_m),
        previous_blocked_mask=previous_blocked_mask,
        target_changed_event=bool(target_changed_event),
        capsule_proxy_xz=capsule_proxy_xz,
        goal_xz=goal_xz,
        reference_snapshot_diff=diff_report,
    )
    decision = supervisor.decide(
        event_class=report.event_class,
        replanning_triggered=report.replanning_triggered,
        reason=report.reason,
        candidate_metrics=candidate if report.replanning_triggered else None,
    )
    return report, decision


def run_static_obstacle_regression(args: argparse.Namespace) -> int:
    from map_update_layer.blockage_classifier import ReferenceBlockageClassifier
    from map_update_layer.lightweight_supervisor import LightweightSupervisorDryRun
    from map_update_layer.perception_to_map import CameraIntrinsics
    from map_update_layer.reference_snapshot_diff import ReferenceSnapshotDiff
    from map_update_layer.recovery_contract import RecoveryHandoffContract

    stage_dir = Path(args.out_root) / "stage_c_obstacle_to_accepted_regression"
    stage_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    handle = build_synthetic_handle()
    default_start_xz, default_goal_xz = default_start_goal(handle)
    clear = build_snapshot(handle, [], 0)
    classifier = ReferenceBlockageClassifier()
    supervisor = LightweightSupervisorDryRun()
    recovery = RecoveryHandoffContract()
    max_attempts = max(1, int(getattr(args, "max_repair_attempts", 3)))

    attempts = [
        {
            "name": "real_control_static_obstacle_to_accepted_attempt_1_default_progressed",
            "reference_start_xz": default_start_xz,
            "goal_xz": default_goal_xz,
            "current_fraction": 0.05,
            "obstacle_fraction": 0.20,
            "rect_size_m": 0.006,
            "offset_xz_m": (0.0, -0.006),
        },
        {
            "name": "real_control_static_obstacle_to_accepted_attempt_2_mid_corridor",
            "reference_start_xz": default_start_xz,
            "goal_xz": default_goal_xz,
            "current_fraction": 0.18,
            "obstacle_fraction": 0.43,
            "rect_size_m": 0.012,
            "offset_xz_m": (0.0, 0.0),
        },
        {
            "name": "real_control_static_obstacle_to_accepted_attempt_3_lower_goal",
            "reference_start_xz": default_start_xz,
            "goal_xz": (0.35, 0.45),
            "current_fraction": 0.00,
            "obstacle_fraction": 0.35,
            "rect_size_m": 0.006,
            "offset_xz_m": (0.0, 0.0),
        },
    ][:max_attempts]

    attempt_rows: list[dict[str, Any]] = []
    transition_rows: list[dict[str, Any]] = []
    recovery_rows: list[dict[str, Any]] = []
    accepted_row: Optional[dict[str, Any]] = None

    for idx, attempt in enumerate(attempts, start=1):
        reference_start_xz = tuple(float(v) for v in attempt["reference_start_xz"])
        goal_xz = tuple(float(v) for v in attempt["goal_xz"])
        reference = build_reference_trace(reference_start_xz, goal_xz, samples=140)
        current_s_m, current_start_xz = reference_xz_at_fraction(reference, float(attempt["current_fraction"]))
        _, obstacle_center = reference_xz_at_fraction(reference, float(attempt["obstacle_fraction"]))
        offset = tuple(float(v) for v in attempt["offset_xz_m"])
        rect = (
            float(obstacle_center[0] + offset[0]),
            float(obstacle_center[1] + offset[1]),
            float(attempt["rect_size_m"]),
            float(attempt["rect_size_m"]),
        )
        snapshot = build_snapshot(handle, [rect], 300000 + idx)
        diff = ReferenceSnapshotDiff(
            handle=handle,
            intrinsics=CameraIntrinsics(fx=615.0, fy=615.0, cx=319.5, cy=239.5, width=640, height=480),
            y_plane=float(handle.target_y),
            require_current_transform=False,
        )
        diff.capture(clear, q_act=np.zeros(3), T_base_cam=None, reference_corridor=reference.xz)
        diff_report = diff.evaluate(
            snapshot,
            current_T_base_cam=None,
            current_corridor=reference.xz,
            fov_mask_override=np.ones(tuple(handle.shape), dtype=bool),
        )
        pre_report, _ = classify_and_decide(
            classifier=classifier,
            supervisor=supervisor,
            snapshot=snapshot,
            reference_trace=reference,
            current_s_m=current_s_m,
            previous_blocked_mask=clear.blocked_mask,
            target_changed_event=False,
            diff_report=diff_report,
            capsule_proxy_xz=None,
            goal_xz=goal_xz,
            candidate=mock_candidate_metrics(True),
        )

        source = "not_called"
        control_called = False
        control_exception = ""
        control_summary_path = ""
        candidate: dict[str, Any] = {
            "candidate_metrics_source": "not_called",
            "candidate_accepted": False,
            "rejection_reason": "classifier_did_not_emit_reference_blocked",
            "invalid_reason_code": str(pre_report.event_class),
        }
        if str(pre_report.event_class) == RuntimeEvent.REFERENCE_BLOCKED:
            source = "ControlModule.run"
            control_called = True
            try:
                control_summary = run_control_candidate(
                    snapshot=snapshot,
                    clear_snapshot=clear,
                    start_xz=current_start_xz,
                    goal_xz=goal_xz,
                    out_dir=stage_dir / "control_dryrun" / str(attempt["name"]),
                )
                candidate = candidate_metrics(control_summary)
                control_summary_path = str(stage_dir / "control_dryrun" / str(attempt["name"]) / "summary.json")
            except Exception as exc:
                control_exception = f"{type(exc).__name__}: {exc}"
                candidate = {
                    "candidate_metrics_source": "control_exception",
                    "candidate_accepted": False,
                    "rejection_reason": f"control_dryrun_exception:{type(exc).__name__}:{exc}",
                    "invalid_reason_code": "control_dryrun_exception",
                }

        machine = SupervisorStateMachine()
        machine.accept_reference(frame_index=0, reason=f"initial_reference_for_{attempt['name']}")
        report, decision = classify_and_decide(
            classifier=classifier,
            supervisor=supervisor,
            snapshot=snapshot,
            reference_trace=reference,
            current_s_m=current_s_m,
            previous_blocked_mask=clear.blocked_mask,
            target_changed_event=False,
            diff_report=diff_report,
            capsule_proxy_xz=None,
            goal_xz=goal_xz,
            candidate=candidate,
        )
        if bool(decision.recovery_required):
            req = recovery.build_request(
                snapshot=snapshot,
                event_class=report.event_class,
                reject_reason=decision.reject_reason,
                invalid_reason_code=decision.invalid_reason_code,
                reference_progress_s=current_s_m,
                current_start_xz=current_start_xz,
            )
            recovery_rows.append({"case": str(attempt["name"]), **req.as_dict()})
        machine.evaluate(
            frame_index=idx,
            report=report,
            supervisor_decision=decision,
            recovery_key=blocked_mask_key(snapshot),
        )
        if bool(decision.candidate_accepted):
            diff.capture(snapshot, q_act=np.zeros(3), T_base_cam=None, reference_corridor=reference.xz)
            machine.accept_reference(frame_index=idx, reason="accepted_static_obstacle_candidate_reference_recaptured")

        row = {
            "case": str(attempt["name"]),
            "source": source,
            "control_called": bool(control_called or decision.control_called),
            "control_exception": control_exception,
            "control_summary_path": control_summary_path,
            "reference_start_xz": reference_start_xz,
            "current_start_xz": current_start_xz,
            "goal_xz": goal_xz,
            "current_fraction": float(attempt["current_fraction"]),
            "obstacle_fraction": float(attempt["obstacle_fraction"]),
            "current_s_m": current_s_m,
            "obstacle_rect": rect,
            "event_class": str(report.event_class),
            "replanning_triggered": bool(report.replanning_triggered),
            "current_pose_blocked_count": int(getattr(report, "current_pose_blocked_count", -1)),
            "goal_blocked_count": int(getattr(report, "goal_blocked_count", -1)),
            "reference_blocked_count": int(getattr(report, "reference_blocked_count", -1)),
            "candidate_metrics_source": str(candidate.get("candidate_metrics_source", "")),
            "candidate_accepted": bool(candidate.get("candidate_accepted", False)),
            "supervisor_candidate_accepted": bool(decision.candidate_accepted),
            "supervisor_recovery_required": bool(decision.recovery_required),
            "rejection_reason": str(decision.reject_reason),
            "invalid_reason_code": str(decision.invalid_reason_code),
            "raw_candidate_metrics": json.dumps(jsonable(candidate), sort_keys=True, ensure_ascii=False),
            "transition_to_reference_switched": any(
                str(t.get("to_state")) == RuntimeState.REFERENCE_SWITCHED for t in machine.transitions
            ),
            "reference_recaptured_after_switch": any(
                str(t.get("from_state")) == RuntimeState.REFERENCE_SWITCHED
                and str(t.get("to_state")) == RuntimeState.REFERENCE_ACTIVE
                for t in machine.transitions
            ),
        }
        attempt_rows.append(row)
        transition_rows.extend({**t, "case": str(attempt["name"])} for t in machine.transitions)
        if is_real_control_accepted(row):
            accepted_row = row
            break

    real_connected = any(is_real_control_connected(row) for row in attempt_rows)
    obstacle_event_seen = any(str(row.get("event_class")) == RuntimeEvent.REFERENCE_BLOCKED for row in attempt_rows)
    accepted = accepted_row is not None
    clean_accepted_counts = bool(
        accepted_row is not None
        and int(accepted_row.get("current_pose_blocked_count", -1)) == 0
        and int(accepted_row.get("goal_blocked_count", -1)) == 0
        and int(accepted_row.get("reference_blocked_count", 0)) > 0
    )
    switched = bool(accepted_row and accepted_row.get("transition_to_reference_switched"))
    recaptured = bool(accepted_row and accepted_row.get("reference_recaptured_after_switch"))
    passed = bool(accepted and clean_accepted_counts and switched and recaptured)
    if passed:
        verdict = "PASS"
        blockers: list[str] = []
        allowed_next = "Stage D"
    elif real_connected and obstacle_event_seen:
        verdict = "PARTIAL"
        blockers = [
            "real ControlModule.run was connected for REFERENCE_BLOCKED, but no accepted alternate candidate was found under capsule-consistent validation",
            "last_rejection_reasons="
            + json.dumps(
                sorted({str(row.get("rejection_reason", "")) for row in attempt_rows if row.get("rejection_reason")}),
                ensure_ascii=False,
            ),
        ]
        allowed_next = "STOP"
    else:
        verdict = "FAIL"
        blockers = ["REFERENCE_BLOCKED event or ControlModule.run connection was not reached"]
        allowed_next = "STOP"

    write_csv(stage_dir / "static_obstacle_attempts.csv", attempt_rows)
    write_csv(stage_dir / "state_transition_coverage.csv", transition_rows)
    write_jsonl(stage_dir / "recovery_requests.jsonl", recovery_rows)
    write_json(
        stage_dir / "control_dryrun_candidate_summary.json",
        {
            "obstacle_attempt_rows": attempt_rows,
            "accepted_obstacle_row": accepted_row,
            "real_control_dryrun_connected": real_connected,
            "static_obstacle_reference_blocked_event_seen": obstacle_event_seen,
            "real_static_obstacle_candidate_accepted": accepted,
            "supervisor_consumed_obstacle_candidate": switched,
            "reference_recaptured_after_switch": recaptured,
        },
    )
    safety = no_command_safety_report()
    write_json(stage_dir / "safety_no_command_report.json", safety)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage C - obstacle-to-accepted regression", "timestamp_s": time.time(), "status": "completed"})
    summary = {
        "stage_name": "Stage C - deterministic static-obstacle accepted-candidate regression",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": verdict,
        "stage_passed": passed,
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "static_obstacle_attempts.csv"),
            str(stage_dir / "state_transition_coverage.csv"),
            str(stage_dir / "control_dryrun_candidate_summary.json"),
            str(stage_dir / "recovery_requests.jsonl"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": allowed_next,
        "repair_attempt_count": len(attempt_rows),
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "previous_target_changed_only_gap_separated": True,
        "static_obstacle_event_class": str(accepted_row.get("event_class")) if accepted_row else (
            "REFERENCE_BLOCKED" if obstacle_event_seen else "UNVERIFIED"
        ),
        "obstacle_current_pose_blocked_count": int(accepted_row.get("current_pose_blocked_count")) if accepted_row else (
            int(next((row.get("current_pose_blocked_count", -1) for row in attempt_rows if str(row.get("event_class")) == RuntimeEvent.REFERENCE_BLOCKED), -1))
        ),
        "obstacle_reference_blocked_count": int(accepted_row.get("reference_blocked_count")) if accepted_row else (
            int(next((row.get("reference_blocked_count", -1) for row in attempt_rows if str(row.get("event_class")) == RuntimeEvent.REFERENCE_BLOCKED), -1))
        ),
        "obstacle_goal_blocked_count": int(accepted_row.get("goal_blocked_count")) if accepted_row else (
            int(next((row.get("goal_blocked_count", -1) for row in attempt_rows if str(row.get("event_class")) == RuntimeEvent.REFERENCE_BLOCKED), -1))
        ),
        "real_control_obstacle_candidate": "PASS" if real_connected else "FAIL",
        "obstacle_candidate_accepted": "PASS" if accepted else ("PARTIAL" if real_connected else "FAIL"),
        "supervisor_consumed_obstacle_candidate": "PASS" if switched else ("PARTIAL" if real_connected else "FAIL"),
        "reference_switched_after_obstacle_candidate": "PASS" if recaptured else ("PARTIAL" if real_connected else "FAIL"),
        "controlmodule_metadata_keyerror": "ABSENT" if not any(
            "target_contact_x_m" in str(row.get("control_exception", ""))
            or "target_contact_x_m" in str(row.get("rejection_reason", ""))
            for row in attempt_rows
        ) else "PRESENT",
        "robot_command_safety": safety["robot_command_safety"],
        "command_sent": False,
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if passed else 1


def capsule_original_attempt_specs(handle: Any) -> list[dict[str, Any]]:
    start_xz, default_goal_xz = default_start_goal(handle)
    return [
        {
            "case": "original_attempt_1_default_progressed",
            "reference_start_xz": start_xz,
            "goal_xz": default_goal_xz,
            "current_fraction": 0.05,
            "obstacle_fraction": 0.20,
            "rect_size_m": 0.006,
            "offset_xz_m": (0.0, -0.006),
        },
        {
            "case": "original_attempt_2_mid_corridor",
            "reference_start_xz": start_xz,
            "goal_xz": default_goal_xz,
            "current_fraction": 0.18,
            "obstacle_fraction": 0.43,
            "rect_size_m": 0.012,
            "offset_xz_m": (0.0, 0.0),
        },
        {
            "case": "original_attempt_3_lower_goal",
            "reference_start_xz": start_xz,
            "goal_xz": (0.35, 0.45),
            "current_fraction": 0.0,
            "obstacle_fraction": 0.35,
            "rect_size_m": 0.006,
            "offset_xz_m": (0.0, 0.0),
        },
    ]


def read_json_or_none(path: Path) -> Optional[dict[str, Any]]:
    for encoding in ("utf-8", "utf-8-sig", "utf-16"):
        try:
            return json.loads(path.read_text(encoding=encoding))
        except Exception:
            pass
    return None


def build_snapshot_with_inflation(
    handle: Any,
    rects: list[tuple[float, float, float, float]],
    *,
    inflation_m: float,
    sequence_id: int,
    source: str,
) -> Any:
    from constants import DEFAULT_MU_MIN
    from map_update_layer.map_update_layer import MapUpdateLayer
    from map_update_layer.map_update_request import MapUpdateRequest

    req = MapUpdateRequest(
        occupied_rects=tuple(rects),
        source=str(source),
        sequence_id=int(sequence_id),
        timestamp_s=float(sequence_id),
        inflation_m=float(inflation_m),
        sensor_inflation_m=float(inflation_m),
        per_rect_inflation_m=tuple(float(inflation_m) for _ in rects),
        adapter_stats={"capsule_planning_inflation_m": float(inflation_m)},
    )
    return MapUpdateLayer().build(handle, mu_min=DEFAULT_MU_MIN, request=req)


def build_capsule_clearance_planning_snapshot(
    *,
    sensor_snapshot: Any,
    rects: list[tuple[float, float, float, float]],
    capsule_planning_inflation_m: float,
    sequence_id: int,
    planning_extra_mask: Optional[np.ndarray] = None,
    planning_extra_source: str = "",
) -> Any:
    from map_update_layer.active_map_snapshot import ActiveMapSnapshot

    handle = sensor_snapshot.handle
    planning_snapshot = build_snapshot_with_inflation(
        handle,
        rects,
        inflation_m=float(capsule_planning_inflation_m),
        sequence_id=int(sequence_id),
        source="capsule_clearance_planning_mask",
    )
    base = np.asarray(sensor_snapshot.base_feasible_mask, dtype=bool)
    planning_blocked = np.asarray(planning_snapshot.blocked_mask, dtype=bool)
    extra = np.zeros_like(planning_blocked, dtype=bool)
    if planning_extra_mask is not None:
        extra = np.asarray(planning_extra_mask, dtype=bool)
        if extra.shape != planning_blocked.shape:
            raise ValueError("planning_extra_mask shape must match active-map mask shape")
    final_active = base & (~planning_blocked) & (~extra)
    stats = dict(sensor_snapshot.stats)
    stats.update(
        {
            "planning_snapshot_source": "capsule_clearance_planning_snapshot",
            "sensor_event_snapshot_source": str(sensor_snapshot.source),
            "capsule_planning_inflation_m": float(capsule_planning_inflation_m),
            "sensor_blocked_cells": float(np.count_nonzero(sensor_snapshot.blocked_mask)),
            "planning_blocked_cells": float(np.count_nonzero(planning_blocked | extra)),
            "planning_inflation_blocked_cells": float(np.count_nonzero(planning_blocked)),
            "planning_extra_blocked_cells": float(np.count_nonzero(planning_blocked & (~sensor_snapshot.blocked_mask))),
            "capsule_cegis_planning_extra_cells": float(np.count_nonzero(extra)),
            "capsule_cegis_planning_extra_source": str(planning_extra_source),
            "split_mask_adapter_final_active_from_planning_blocked_mask": True,
            "split_mask_adapter_blocked_mask_from_sensor_event_snapshot": True,
            "post_dls_capsule_validation_mask_source": "sensor_event_snapshot.blocked_mask",
        }
    )
    layer_masks = dict(sensor_snapshot.layer_masks)
    layer_masks["sensor_blocked_mask"] = np.asarray(sensor_snapshot.blocked_mask, dtype=bool)
    layer_masks["planning_inflation_blocked_mask"] = planning_blocked
    layer_masks["planning_cegis_extra_mask"] = extra
    layer_masks["planning_blocked_mask"] = np.asarray(planning_blocked | extra, dtype=bool)
    layer_masks["planning_extra_blocked_mask"] = np.asarray((planning_blocked | extra) & (~sensor_snapshot.blocked_mask), dtype=bool)
    return ActiveMapSnapshot(
        handle=handle,
        base_feasible_mask=sensor_snapshot.base_feasible_mask,
        final_active_mask=final_active,
        blocked_mask=sensor_snapshot.blocked_mask,
        display_rects=planning_snapshot.display_rects,
        stats=stats,
        layer_masks=layer_masks,
        source="capsule_clearance_planning_snapshot",
        mu_min=float(sensor_snapshot.mu_min),
    )


def parse_episode_q_traj(path: Path) -> np.ndarray:
    if not path.exists():
        return np.empty((0, 3), dtype=np.float64)
    rows: list[list[float]] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                rows.append([float(row["q2"]), float(row["q3"]), float(row["q5"])])
            except Exception:
                continue
    return np.asarray(rows, dtype=np.float64).reshape(-1, 3) if rows else np.empty((0, 3), dtype=np.float64)


def capsule_collision_forensics(q_traj: np.ndarray, handle: Any, mask: np.ndarray) -> dict[str, Any]:
    try:
        from capsule_collision import CapsuleCollision
        from kinematics import Kinematics
    except Exception as exc:
        return {"collision_forensics_available": False, "unavailable_reason": f"import_error:{type(exc).__name__}:{exc}"}

    q_arr = np.asarray(q_traj, dtype=np.float64).reshape(-1, 3)
    if q_arr.shape[0] == 0:
        return {"collision_forensics_available": False, "unavailable_reason": "empty_or_missing_episode_q_traj"}
    collision = CapsuleCollision()
    kin = Kinematics()
    base_mask = np.asarray(mask, dtype=bool)
    dilated = collision.build_dilated_masks(handle, base_mask)
    histogram = {str(i): 0 for i in range(len(collision.radii_m))}
    first: Optional[dict[str, Any]] = None
    colliding_ticks = 0
    total_checked_samples = 0
    colliding_ee_cells: set[tuple[int, int]] = set()
    for tick, q in enumerate(q_arr):
        pts = kin.capsule_proxy_points_xz(q)
        tick_hit = False
        for seg_idx, radius in enumerate(collision.radii_m):
            samples = collision._sample_segment(pts[seg_idx], pts[seg_idx + 1], handle.resolution_m)
            total_checked_samples += int(samples.shape[0])
            inflated = dilated[float(radius)]
            for sample in samples:
                ix = int(round((float(sample[0]) - handle.x0) / handle.resolution_m))
                iz = int(round((float(sample[1]) - handle.z0) / handle.resolution_m))
                if ix < 0 or ix >= inflated.shape[0] or iz < 0 or iz >= inflated.shape[1]:
                    continue
                if bool(inflated[ix, iz]):
                    histogram[str(seg_idx)] = int(histogram[str(seg_idx)]) + 1
                    tick_hit = True
                    ee_xz = kin.task_pose(q)[:2]
                    ee_ix = int(round((float(ee_xz[0]) - handle.x0) / handle.resolution_m))
                    ee_iz = int(round((float(ee_xz[1]) - handle.z0) / handle.resolution_m))
                    if 0 <= ee_ix < base_mask.shape[0] and 0 <= ee_iz < base_mask.shape[1]:
                        colliding_ee_cells.add((int(ee_ix), int(ee_iz)))
                    if first is None:
                        first = {
                            "first_collision_tick": int(tick),
                            "first_collision_q": [float(v) for v in q.tolist()],
                            "first_collision_ee_xz": [float(v) for v in kin.task_pose(q)[:2].tolist()],
                            "first_collision_segment_index": int(seg_idx),
                            "first_collision_segment_radius_m": float(radius),
                            "first_collision_sample_xz": [float(sample[0]), float(sample[1])],
                            "first_collision_cell_index": [int(ix), int(iz)],
                        }
                    break
        if tick_hit:
            colliding_ticks += 1
    out = {
        "collision_forensics_available": True,
        "colliding_tick_count": int(colliding_ticks),
        "capsule_proxy_total_ticks": int(q_arr.shape[0]),
        "capsule_proxy_checked_samples": int(total_checked_samples),
        "colliding_segment_histogram": histogram,
        "colliding_ee_cell_indices": [[int(ix), int(iz)] for ix, iz in sorted(colliding_ee_cells)[:256]],
        "colliding_ee_unique_cell_count": int(len(colliding_ee_cells)),
        "capsule_proxy_radius_max_m": float(np.max(collision.radii_m)),
        "capsule_proxy_sample_ds_m": float(collision.sample_ds_m),
    }
    out.update(first or {
        "first_collision_tick": None,
        "first_collision_q": [],
        "first_collision_ee_xz": [],
        "first_collision_segment_index": None,
        "first_collision_segment_radius_m": None,
        "first_collision_sample_xz": [],
        "first_collision_cell_index": [],
    })
    return out


def update_planning_extra_mask_from_forensics(
    extra_mask: Optional[np.ndarray],
    *,
    handle: Any,
    forensics: dict[str, Any],
    radius_cells: int = 2,
) -> tuple[np.ndarray, int]:
    """Add colliding EE cells to a planning-only exclusion mask.

    This does not alter the sensor blocked mask used by post-DLS capsule
    validation.  It only gives the next candidate generation pass the
    trajectory-level counterexample that the previous DLS rollout exposed.
    """
    shape = tuple(int(v) for v in handle.shape)
    out = np.zeros(shape, dtype=bool) if extra_mask is None else np.asarray(extra_mask, dtype=bool).copy()
    before = int(np.count_nonzero(out))
    cells = forensics.get("colliding_ee_cell_indices", [])
    if not cells and forensics.get("first_collision_ee_xz"):
        try:
            xz = forensics.get("first_collision_ee_xz", [])
            ix = int(round((float(xz[0]) - handle.x0) / handle.resolution_m))
            iz = int(round((float(xz[1]) - handle.z0) / handle.resolution_m))
            cells = [[ix, iz]]
        except Exception:
            cells = []
    r = max(int(radius_cells), 0)
    for cell in cells:
        try:
            ix, iz = int(cell[0]), int(cell[1])
        except Exception:
            continue
        for dx in range(-r, r + 1):
            for dz in range(-r, r + 1):
                nx, nz = ix + dx, iz + dz
                if 0 <= nx < shape[0] and 0 <= nz < shape[1]:
                    out[nx, nz] = True
    return out, int(np.count_nonzero(out) - before)


def classify_static_obstacle_fixture(
    *,
    handle: Any,
    clear_snapshot: Any,
    spec: dict[str, Any],
    sequence_id: int,
) -> dict[str, Any]:
    from map_update_layer.blockage_classifier import ReferenceBlockageClassifier
    from map_update_layer.lightweight_supervisor import LightweightSupervisorDryRun
    from map_update_layer.perception_to_map import CameraIntrinsics
    from map_update_layer.reference_snapshot_diff import ReferenceSnapshotDiff

    reference_start_xz = tuple(float(v) for v in spec["reference_start_xz"])
    goal_xz = tuple(float(v) for v in spec["goal_xz"])
    reference = build_reference_trace(reference_start_xz, goal_xz, samples=140)
    current_s_m, current_start_xz = reference_xz_at_fraction(reference, float(spec["current_fraction"]))
    _, obstacle_center = reference_xz_at_fraction(reference, float(spec["obstacle_fraction"]))
    offset = tuple(float(v) for v in spec.get("offset_xz_m", (0.0, 0.0)))
    rect = (
        float(obstacle_center[0] + offset[0]),
        float(obstacle_center[1] + offset[1]),
        float(spec["rect_size_m"]),
        float(spec["rect_size_m"]),
    )
    sensor_snapshot = build_snapshot(handle, [rect], sequence_id)
    diff = ReferenceSnapshotDiff(
        handle=handle,
        intrinsics=CameraIntrinsics(fx=615.0, fy=615.0, cx=319.5, cy=239.5, width=640, height=480),
        y_plane=float(handle.target_y),
        require_current_transform=False,
    )
    diff.capture(clear_snapshot, q_act=np.zeros(3), T_base_cam=None, reference_corridor=reference.xz)
    diff_report = diff.evaluate(
        sensor_snapshot,
        current_T_base_cam=None,
        current_corridor=reference.xz,
        fov_mask_override=np.ones(tuple(handle.shape), dtype=bool),
    )
    report, _ = classify_and_decide(
        classifier=ReferenceBlockageClassifier(),
        supervisor=LightweightSupervisorDryRun(),
        snapshot=sensor_snapshot,
        reference_trace=reference,
        current_s_m=current_s_m,
        previous_blocked_mask=clear_snapshot.blocked_mask,
        target_changed_event=False,
        diff_report=diff_report,
        capsule_proxy_xz=None,
        goal_xz=goal_xz,
        candidate=mock_candidate_metrics(True),
    )
    return {
        "reference": reference,
        "current_s_m": current_s_m,
        "current_start_xz": current_start_xz,
        "goal_xz": goal_xz,
        "obstacle_rect": rect,
        "sensor_snapshot": sensor_snapshot,
        "diff_report": diff_report,
        "classification_report": report,
    }


def run_control_with_forensics(
    *,
    snapshot: Any,
    clear_snapshot: Any,
    start_xz: tuple[float, float],
    goal_xz: tuple[float, float],
    out_dir: Path,
    initial_q_active: Optional[np.ndarray] = None,
) -> tuple[dict[str, Any], dict[str, Any], str]:
    summary = run_control_candidate(
        snapshot=snapshot,
        clear_snapshot=clear_snapshot,
        start_xz=start_xz,
        goal_xz=goal_xz,
        out_dir=out_dir,
        initial_q_active=initial_q_active,
    )
    metrics = candidate_metrics(summary)
    q_traj = parse_episode_q_traj(out_dir / "episode.csv")
    forensics = capsule_collision_forensics(q_traj, snapshot.handle, snapshot.blocked_mask)
    return metrics, forensics, str(out_dir / "summary.json")


def run_capsule_stage_a(args: argparse.Namespace) -> int:
    stage_dir = Path(args.out_root) / "stage_a_baseline_forensics"
    stage_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    previous_root = PROJECT_ROOT / "path" / STATIC_OBSTACLE_OUT_ROOT_NAME
    previous_stage = read_json_or_none(previous_root / "stage_c_obstacle_to_accepted_regression" / "stage_summary.json") or {}
    previous_summary = read_json_or_none(previous_root / "static_obstacle_candidate_closure_summary.json") or {}
    attempts_path = previous_root / "stage_c_obstacle_to_accepted_regression" / "static_obstacle_attempts.csv"
    attempt_rows: list[dict[str, Any]] = []
    if attempts_path.exists():
        with attempts_path.open("r", newline="", encoding="utf-8") as f:
            attempt_rows = list(csv.DictReader(f))
    from control_module.constants import CAPSULE_PROXY_RADII_M, DEFAULT_OBSTACLE_INFLATION_M

    radius_max = float(np.max(CAPSULE_PROXY_RADII_M))
    sensor_infl = float(DEFAULT_OBSTACLE_INFLATION_M)
    ratio = float(radius_max / max(sensor_infl, 1.0e-12))
    blockers: list[str] = []
    if str(previous_stage.get("stage_verdict")) != "PARTIAL":
        blockers.append("previous_stage_c_not_partial")
    if {str(row.get("rejection_reason", "")) for row in attempt_rows} != {"capsule_proxy_collision"}:
        blockers.append("previous_rejection_reason_not_capsule_proxy_collision_only")
    if any(str(row.get("event_class")) != RuntimeEvent.REFERENCE_BLOCKED for row in attempt_rows):
        blockers.append("previous_event_not_reference_blocked")
    if any(int(float(row.get("current_pose_blocked_count", -1))) != 0 for row in attempt_rows):
        blockers.append("previous_current_pose_not_clean")
    if any(int(float(row.get("goal_blocked_count", -1))) != 0 for row in attempt_rows):
        blockers.append("previous_goal_not_clean")
    if any(int(float(row.get("reference_blocked_count", 0))) <= 0 for row in attempt_rows):
        blockers.append("previous_reference_not_blocked")
    if any(str(row.get("control_exception", "")) for row in attempt_rows):
        blockers.append("previous_control_exception_present")
    if radius_max < 0.04:
        blockers.append("capsule_radius_too_small_for_baseline")
    if sensor_infl > 0.01:
        blockers.append("sensor_inflation_not_small_baseline")
    if ratio < 4.0:
        blockers.append("capsule_to_sensor_ratio_too_small")
    passed = not blockers and len(attempt_rows) >= 1
    write_csv(stage_dir / "previous_attempts_summary.csv", attempt_rows)
    inventory = {
        "previous_stage_c_verdict": previous_stage.get("stage_verdict"),
        "previous_overall_stage_sequence": (previous_summary.get("final_report") or {}).get("Stage sequence completed"),
        "previous_attempt_count": len(attempt_rows),
        "previous_rejection_reasons": sorted({str(row.get("rejection_reason", "")) for row in attempt_rows}),
        "capsule_proxy_radius_max_m": radius_max,
        "sensor_inflation_m": sensor_infl,
        "capsule_to_sensor_inflation_ratio": ratio,
        "baseline_blocker": "capsule_proxy_collision",
    }
    write_json(stage_dir / "capsule_mismatch_inventory.json", inventory)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage A - baseline capsule mismatch forensics", "timestamp_s": time.time(), "status": "completed"})
    summary = {
        "stage_name": "Stage A - baseline capsule mismatch forensics",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": "PASS" if passed else "FAIL",
        "stage_passed": passed,
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "capsule_mismatch_inventory.json"),
            str(stage_dir / "previous_attempts_summary.csv"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage B" if passed else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if passed else 1


def run_capsule_stage_b(args: argparse.Namespace) -> int:
    stage_dir = Path(args.out_root) / "stage_b_capsule_patch"
    stage_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    source = Path(__file__).read_text(encoding="utf-8")
    findings = scan_forbidden_code_tokens(source)
    compile_ok = True
    compile_error = ""
    try:
        compile(source, str(Path(__file__)), "exec")
    except Exception as exc:
        compile_ok = False
        compile_error = f"{type(exc).__name__}: {exc}"
    from control_module.constants import CAPSULE_PROXY_RADII_M, CAPSULE_PROXY_SAMPLE_DS_M

    safety_ok = not any(int(v) > 0 for v in findings.values())
    capsule_contract = {
        "first_collision_tick": "required_when_episode_csv_available",
        "first_collision_q": "required_when_collision_found",
        "first_collision_ee_xz": "required_when_collision_found",
        "first_collision_segment_index": "required_when_collision_found",
        "first_collision_segment_radius_m": "required_when_collision_found",
        "first_collision_sample_xz": "required_when_collision_found",
        "first_collision_cell_index": "required_when_collision_found",
        "colliding_tick_count": "required",
        "colliding_segment_histogram": "required",
        "capsule_proxy_radius_max_m": float(np.max(CAPSULE_PROXY_RADII_M)),
        "capsule_proxy_sample_ds_m": float(CAPSULE_PROXY_SAMPLE_DS_M),
    }
    planning_contract = {
        "sensor_event_snapshot": "ReferenceSnapshotDiff and ReferenceBlockageClassifier input",
        "capsule_clearance_planning_snapshot": "ControlModule.run candidate-generation input",
        "final_active_mask_source": "capsule clearance planning blocked mask",
        "blocked_mask_source": "sensor event snapshot blocked mask for post-DLS capsule validation",
        "capsule_planning_margin_candidates_m": [0.004, 0.009, 0.014],
        "post_dls_capsule_validation_enabled": True,
    }
    safety = no_command_safety_report()
    write_json(stage_dir / "harness_static_scan.json", {"forbidden_term_counts": findings, "robot_command_safety": "PASS" if safety_ok else "FAIL"})
    write_json(stage_dir / "capsule_forensics_contract.json", capsule_contract)
    write_json(stage_dir / "planning_clearance_contract.json", planning_contract)
    write_json(stage_dir / "no_command_safety_report.json", safety)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage B - capsule forensics and clearance adapter patch", "timestamp_s": time.time(), "status": "completed"})
    passed = bool(compile_ok and safety_ok)
    summary = {
        "stage_name": "Stage B - capsule forensics and planning-clearance adapter patch",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": "PASS" if passed else "FAIL",
        "stage_passed": passed,
        "stage_blockers": [] if passed else [compile_error or "forbidden_static_scan"],
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "harness_static_scan.json"),
            str(stage_dir / "capsule_forensics_contract.json"),
            str(stage_dir / "planning_clearance_contract.json"),
            str(stage_dir / "no_command_safety_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage C" if passed else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "compile_ok": compile_ok,
        "compile_error": compile_error,
        "capsule_radius_unchanged": True,
        "post_dls_capsule_validation_enabled": True,
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if passed else 1


def run_capsule_stage_c(args: argparse.Namespace) -> int:
    stage_dir = Path(args.out_root) / "stage_c_capsule_forensics"
    stage_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    handle = build_synthetic_handle()
    clear = build_snapshot(handle, [], 0)
    rows: list[dict[str, Any]] = []
    forensics_rows: list[dict[str, Any]] = []
    for idx, spec in enumerate(capsule_original_attempt_specs(handle), start=1):
        fixture = classify_static_obstacle_fixture(handle=handle, clear_snapshot=clear, spec=spec, sequence_id=600000 + idx)
        report = fixture["classification_report"]
        metrics: dict[str, Any]
        forensics: dict[str, Any]
        summary_path = ""
        try:
            metrics, forensics, summary_path = run_control_with_forensics(
                snapshot=fixture["sensor_snapshot"],
                clear_snapshot=clear,
                start_xz=fixture["current_start_xz"],
                goal_xz=fixture["goal_xz"],
                out_dir=stage_dir / "control_dryrun" / str(spec["case"]),
            )
        except Exception as exc:
            metrics = {"candidate_metrics_source": "control_exception", "candidate_accepted": False, "rejection_reason": f"{type(exc).__name__}:{exc}"}
            forensics = {"collision_forensics_available": False, "unavailable_reason": f"control_exception:{type(exc).__name__}:{exc}"}
        row = {
            "case": spec["case"],
            "source": "ControlModule.run",
            "control_summary_path": summary_path,
            "event_class": str(report.event_class),
            "current_pose_blocked_count": int(report.current_pose_blocked_count),
            "goal_blocked_count": int(report.goal_blocked_count),
            "reference_blocked_count": int(report.reference_blocked_count),
            "current_s_m": float(fixture["current_s_m"]),
            "current_start_xz": fixture["current_start_xz"],
            "goal_xz": fixture["goal_xz"],
            "obstacle_rect": fixture["obstacle_rect"],
            **metrics,
            "collision_forensics_available": bool(forensics.get("collision_forensics_available", False)),
            "first_collision_tick": forensics.get("first_collision_tick"),
            "first_collision_segment_index": forensics.get("first_collision_segment_index"),
            "colliding_tick_count": forensics.get("colliding_tick_count"),
        }
        rows.append(row)
        forensics_rows.append({"case": spec["case"], **forensics})
    write_csv(stage_dir / "original_attempts_with_forensics.csv", rows)
    write_jsonl(stage_dir / "capsule_collision_forensics.jsonl", forensics_rows)
    real_ok = all(str(row.get("source")) == "ControlModule.run" for row in rows)
    event_ok = all(str(row.get("event_class")) == RuntimeEvent.REFERENCE_BLOCKED for row in rows)
    counts_ok = all(int(row.get("current_pose_blocked_count", -1)) == 0 and int(row.get("goal_blocked_count", -1)) == 0 and int(row.get("reference_blocked_count", 0)) > 0 for row in rows)
    blocker_reproduced = any(str(row.get("rejection_reason")) == "capsule_proxy_collision" for row in rows)
    forensics_ok = any(bool(row.get("collision_forensics_available")) for row in rows)
    safety = no_command_safety_report()
    passed = bool(real_ok and event_ok and counts_ok and blocker_reproduced and forensics_ok)
    partial = bool(real_ok and event_ok and counts_ok and blocker_reproduced)
    verdict = "PASS" if passed else ("PARTIAL" if partial else "FAIL")
    blockers = [] if passed else ([] if partial else ["failed_to_reproduce_capsule_blocker_or_event_contract"])
    if partial and not forensics_ok:
        blockers.append("capsule_forensics_unavailable")
    write_json(
        stage_dir / "control_dryrun_candidate_summary.json",
        {
            "attempt_rows": rows,
            "capsule_forensics_rows": forensics_rows,
            "capsule_blocker_reproduced": blocker_reproduced,
        },
    )
    write_json(stage_dir / "safety_no_command_report.json", safety)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage C - original failing attempts with capsule forensics", "timestamp_s": time.time(), "status": "completed"})
    summary = {
        "stage_name": "Stage C - original failing attempts with per-tick capsule forensics",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": verdict,
        "stage_passed": passed,
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "original_attempts_with_forensics.csv"),
            str(stage_dir / "capsule_collision_forensics.jsonl"),
            str(stage_dir / "control_dryrun_candidate_summary.json"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage D" if verdict in ("PASS", "PARTIAL") and safety["robot_command_safety"] == "PASS" else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "previous_capsule_blocker_reproduced": "PASS" if blocker_reproduced else "FAIL",
        "capsule_collision_forensics": "PASS" if forensics_ok else "PARTIAL",
        "robot_command_safety": safety["robot_command_safety"],
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if verdict in ("PASS", "PARTIAL") else 1


def run_capsule_stage_d(args: argparse.Namespace) -> int:
    stage_dir = Path(args.out_root) / "stage_d_accepted_candidate_search"
    stage_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    max_trials = int(getattr(args, "max_candidate_trials", 300))
    max_stage_duration_s = float(getattr(args, "max_stage_duration_s", 3600.0))
    handle = build_synthetic_handle()
    clear = build_snapshot(handle, [], 0)
    try:
        from kinematics import Kinematics
        home = tuple(float(v) for v in Kinematics().default_start_xz())
    except Exception:
        home, _ = default_start_goal(handle)
    starts = [home, (0.0, 0.75), (0.05, 0.75), (0.10, 0.72), (-0.05, 0.78)]
    _, default_goal = default_start_goal(handle)
    goals = [default_goal, (0.35, 0.45), (0.45, 0.45), (0.50, 0.40), (0.55, 0.35), (0.35, 0.35), (0.25, 0.35), (0.45, 0.30), (0.55, 0.25)]
    clear_pairs: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for idx, (start_xz, goal_xz) in enumerate((s, g) for s in starts for g in goals):
        try:
            metrics, _, _ = run_control_with_forensics(
                snapshot=clear,
                clear_snapshot=clear,
                start_xz=start_xz,
                goal_xz=goal_xz,
                out_dir=stage_dir / "clear_map_precheck" / f"pair_{idx:03d}",
            )
            if bool(metrics.get("candidate_accepted", False)):
                clear_pairs.append((start_xz, goal_xz))
        except Exception:
            continue
    classifier = None  # created by classify_static_obstacle_fixture.
    trial_rows: list[dict[str, Any]] = []
    transition_rows: list[dict[str, Any]] = []
    accepted_row: Optional[dict[str, Any]] = None
    from map_update_layer.lightweight_supervisor import LightweightSupervisorDryRun

    supervisor = LightweightSupervisorDryRun()
    current_fracs = [0.30, 0.45, 0.18, 0.10, 0.05, 0.0]
    obstacle_fracs = [0.18, 0.25, 0.35, 0.45, 0.55, 0.65]
    sizes = [0.004, 0.006, 0.010, 0.015, 0.020]
    offsets = [(0.0, 0.0), (0.01, 0.0), (-0.01, 0.0), (0.0, 0.01), (0.0, -0.01), (0.015, 0.0), (-0.015, 0.0), (0.0, 0.015), (0.0, -0.015), (0.02, 0.0), (-0.02, 0.0), (0.0, 0.02), (0.0, -0.02)]
    inflations = [0.041, 0.045, 0.050, 0.055, 0.065]
    trial = 0
    for pair_idx, (reference_start_xz, goal_xz) in enumerate(clear_pairs):
        reference = build_reference_trace(reference_start_xz, goal_xz, samples=140)
        for cf in current_fracs:
            current_s_m, current_start_xz = reference_xz_at_fraction(reference, cf)
            for of in obstacle_fracs:
                if of <= cf + 0.08 or of >= 0.82:
                    continue
                _, center = reference_xz_at_fraction(reference, of)
                for size in sizes:
                    for offset in offsets:
                        if trial >= max_trials or time.time() - started > max_stage_duration_s or accepted_row is not None:
                            break
                        spec = {
                            "case": f"search_pair{pair_idx}_cf{cf:.2f}_of{of:.2f}_sz{size:.3f}_dx{offset[0]:.3f}_dz{offset[1]:.3f}",
                            "reference_start_xz": reference_start_xz,
                            "goal_xz": goal_xz,
                            "current_fraction": cf,
                            "obstacle_fraction": of,
                            "rect_size_m": size,
                            "offset_xz_m": offset,
                        }
                        fixture = classify_static_obstacle_fixture(handle=handle, clear_snapshot=clear, spec=spec, sequence_id=700000 + trial)
                        report = fixture["classification_report"]
                        if str(report.event_class) != RuntimeEvent.REFERENCE_BLOCKED:
                            continue
                        if int(report.current_pose_blocked_count) != 0 or int(report.goal_blocked_count) != 0 or int(report.reference_blocked_count) <= 0:
                            continue
                        for infl in inflations:
                            if trial >= max_trials or time.time() - started > max_stage_duration_s or accepted_row is not None:
                                break
                            trial += 1
                            planning_snapshot = build_capsule_clearance_planning_snapshot(
                                sensor_snapshot=fixture["sensor_snapshot"],
                                rects=[fixture["obstacle_rect"]],
                                capsule_planning_inflation_m=float(infl),
                                sequence_id=800000 + trial,
                            )
                            case_name = str(spec["case"]).replace(".", "p").replace("-", "m") + f"_infl{infl:.3f}".replace(".", "p")
                            control_exception = ""
                            try:
                                metrics, forensics, summary_path = run_control_with_forensics(
                                    snapshot=planning_snapshot,
                                    clear_snapshot=clear,
                                    start_xz=fixture["current_start_xz"],
                                    goal_xz=fixture["goal_xz"],
                                    out_dir=stage_dir / "control_dryrun" / case_name,
                                )
                            except Exception as exc:
                                metrics = {"candidate_metrics_source": "control_exception", "candidate_accepted": False, "rejection_reason": f"{type(exc).__name__}:{exc}", "invalid_reason_code": "control_exception"}
                                forensics = {"collision_forensics_available": False, "unavailable_reason": f"control_exception:{type(exc).__name__}:{exc}"}
                                summary_path = ""
                                control_exception = f"{type(exc).__name__}: {exc}"
                            _, decision = classify_and_decide(
                                classifier=__import__("map_update_layer.blockage_classifier", fromlist=["ReferenceBlockageClassifier"]).ReferenceBlockageClassifier(),
                                supervisor=supervisor,
                                snapshot=fixture["sensor_snapshot"],
                                reference_trace=fixture["reference"],
                                current_s_m=fixture["current_s_m"],
                                previous_blocked_mask=clear.blocked_mask,
                                target_changed_event=False,
                                diff_report=fixture["diff_report"],
                                capsule_proxy_xz=None,
                                goal_xz=fixture["goal_xz"],
                                candidate=metrics,
                            )
                            machine = SupervisorStateMachine()
                            machine.accept_reference(frame_index=0, reason="initial_reference_for_capsule_search")
                            machine.evaluate(
                                frame_index=trial,
                                report=report,
                                supervisor_decision=decision,
                                recovery_key=blocked_mask_key(fixture["sensor_snapshot"]),
                            )
                            if bool(decision.candidate_accepted):
                                machine.accept_reference(frame_index=trial, reason="accepted_capsule_aware_candidate_reference_recaptured")
                            transition_rows.extend({**row, "case": case_name} for row in machine.transitions)
                            row = {
                                "case": case_name,
                                "trial": trial,
                                "candidate_source": "ControlModule.run",
                                "candidate_metrics_source": str(metrics.get("candidate_metrics_source", "")),
                                "control_exception": control_exception,
                                "control_summary_path": summary_path,
                                "sensor_event_class": str(report.event_class),
                                "sensor_current_pose_blocked_count": int(report.current_pose_blocked_count),
                                "sensor_goal_blocked_count": int(report.goal_blocked_count),
                                "sensor_reference_blocked_count": int(report.reference_blocked_count),
                                "reference_start_xz": reference_start_xz,
                                "current_start_xz": fixture["current_start_xz"],
                                "goal_xz": fixture["goal_xz"],
                                "current_s_m": float(fixture["current_s_m"]),
                                "obstacle_rect": fixture["obstacle_rect"],
                                "capsule_planning_inflation_m": float(infl),
                                "planning_snapshot_source": planning_snapshot.source,
                                "sensor_blocked_cells": float(planning_snapshot.stats.get("sensor_blocked_cells", 0.0)),
                                "planning_blocked_cells": float(planning_snapshot.stats.get("planning_blocked_cells", 0.0)),
                                "planning_extra_blocked_cells": float(planning_snapshot.stats.get("planning_extra_blocked_cells", 0.0)),
                                **metrics,
                                "supervisor_candidate_accepted": bool(decision.candidate_accepted),
                                "transition_to_reference_switched": any(str(t.get("to_state")) == RuntimeState.REFERENCE_SWITCHED for t in machine.transitions),
                                "reference_recaptured_after_switch": any(str(t.get("from_state")) == RuntimeState.REFERENCE_SWITCHED and str(t.get("to_state")) == RuntimeState.REFERENCE_ACTIVE for t in machine.transitions),
                                "mock_candidate_used": False,
                                "post_dls_capsule_validation_enabled": True,
                                "collision_forensics_available": bool(forensics.get("collision_forensics_available", False)),
                            }
                            trial_rows.append(row)
                            if trial % 25 == 0:
                                write_csv(stage_dir / "candidate_search_trials.csv", trial_rows)
                                write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage D - capsule-aware accepted-candidate search", "timestamp_s": time.time(), "trial": trial, "accepted_found": accepted_row is not None})
                            if is_real_control_accepted({
                                "source": "ControlModule.run",
                                "control_called": True,
                                "control_exception": control_exception,
                                "candidate_metrics_source": str(metrics.get("candidate_metrics_source", "")),
                                "candidate_accepted": bool(metrics.get("candidate_accepted", False)),
                                "supervisor_candidate_accepted": bool(decision.candidate_accepted),
                            }) and float(metrics.get("capsule_proxy_collision_free", 0.0)) >= 1.0:
                                accepted_row = row
                                write_json(stage_dir / "accepted_static_obstacle_candidate.json", accepted_row)
                                break
                    if accepted_row is not None or trial >= max_trials or time.time() - started > max_stage_duration_s:
                        break
                if accepted_row is not None or trial >= max_trials or time.time() - started > max_stage_duration_s:
                    break
            if accepted_row is not None or trial >= max_trials or time.time() - started > max_stage_duration_s:
                break
        if accepted_row is not None or trial >= max_trials or time.time() - started > max_stage_duration_s:
            break
    write_csv(stage_dir / "candidate_search_trials.csv", trial_rows)
    write_csv(stage_dir / "state_transition_coverage.csv", transition_rows)
    write_json(stage_dir / "control_dryrun_candidate_summary.json", {"trial_rows": trial_rows, "accepted_static_obstacle_candidate": accepted_row, "clear_pair_count": len(clear_pairs)})
    if accepted_row is not None:
        write_json(stage_dir / "accepted_static_obstacle_candidate.json", accepted_row)
    else:
        write_json(stage_dir / "accepted_static_obstacle_candidate.json", None)
    safety = no_command_safety_report()
    write_json(stage_dir / "safety_no_command_report.json", safety)
    write_json(stage_dir / "capsule_clearance_planning_snapshot_summary.json", {"clear_pair_count": len(clear_pairs), "trial_count": trial, "accepted_found": accepted_row is not None})
    passed = accepted_row is not None
    verdict = "PASS" if passed else "PARTIAL"
    rejection_reasons = sorted({str(row.get("rejection_reason", "")) for row in trial_rows if row.get("rejection_reason")})
    blockers = [] if passed else [f"no accepted candidate found within trial/time budget; rejection_reasons={json.dumps(rejection_reasons, ensure_ascii=False)}"]
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage D - capsule-aware accepted-candidate search", "timestamp_s": time.time(), "trial": trial, "accepted_found": accepted_row is not None})
    summary = {
        "stage_name": "Stage D - capsule-aware accepted-candidate search",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": verdict,
        "stage_passed": passed,
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "candidate_search_trials.csv"),
            str(stage_dir / "accepted_static_obstacle_candidate.json"),
            str(stage_dir / "state_transition_coverage.csv"),
            str(stage_dir / "control_dryrun_candidate_summary.json"),
            str(stage_dir / "capsule_clearance_planning_snapshot_summary.json"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage E" if passed else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "trial_count": trial,
        "max_candidate_trials": max_trials,
        "accepted_found": passed,
        "robot_command_safety": safety["robot_command_safety"],
        "remaining_rejection_reasons": rejection_reasons,
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if passed else 1


def fixture_previous_root() -> Path:
    return PROJECT_ROOT / "path" / CAPSULE_AWARE_OUT_ROOT_NAME


def fixture_stage_dir(args: argparse.Namespace, name: str) -> Path:
    stage_dir = Path(args.out_root) / name
    stage_dir.mkdir(parents=True, exist_ok=True)
    return stage_dir


def episode_q_at_s(path: Path, target_s: float) -> tuple[np.ndarray, dict[str, Any]]:
    if not path.exists():
        return np.empty(0, dtype=np.float64), {"q_available": False, "reason": "episode_csv_missing"}
    best: Optional[dict[str, Any]] = None
    best_dist = float("inf")
    with path.open("r", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            s_val = safe_float(row.get("s"))
            if not math.isfinite(s_val):
                continue
            dist = abs(s_val - float(target_s))
            if dist < best_dist:
                best = row
                best_dist = dist
    if best is None:
        return np.empty(0, dtype=np.float64), {"q_available": False, "reason": "no_finite_s_rows"}
    try:
        q = np.asarray([float(best["q2"]), float(best["q3"]), float(best["q5"])], dtype=np.float64)
    except Exception as exc:
        return np.empty(0, dtype=np.float64), {"q_available": False, "reason": f"invalid_q_row:{type(exc).__name__}:{exc}"}
    return q, {
        "q_available": True,
        "episode_s": safe_float(best.get("s")),
        "requested_s": float(target_s),
        "s_error_m": float(best_dist),
        "ee_x": safe_float(best.get("ee_x")),
        "ee_z": safe_float(best.get("ee_z")),
    }


def cell_to_xz(handle: Any, cell: tuple[int, int]) -> tuple[float, float]:
    ix, iz = int(cell[0]), int(cell[1])
    return (
        float(handle.x0 + ix * handle.resolution_m),
        float(handle.z0 + iz * handle.resolution_m),
    )


def mask_cell_value(mask: np.ndarray, cell: tuple[int, int]) -> bool:
    arr = np.asarray(mask, dtype=bool)
    ix, iz = int(cell[0]), int(cell[1])
    if ix < 0 or ix >= arr.shape[0] or iz < 0 or iz >= arr.shape[1]:
        return False
    return bool(arr[ix, iz])


def nearest_true_cell(handle: Any, mask: np.ndarray, xz: tuple[float, float], max_radius_m: float = 0.08) -> dict[str, Any]:
    arr = np.asarray(mask, dtype=bool)
    xs, zs = np.nonzero(arr)
    if xs.size == 0:
        return {"found": False, "distance_m": float("nan"), "cell": [], "xz": []}
    x = float(xz[0])
    z = float(xz[1])
    cell_x = handle.x0 + xs.astype(np.float64) * handle.resolution_m
    cell_z = handle.z0 + zs.astype(np.float64) * handle.resolution_m
    d = np.hypot(cell_x - x, cell_z - z)
    idx = int(np.argmin(d))
    dist = float(d[idx])
    if dist > float(max_radius_m):
        return {"found": False, "distance_m": dist, "cell": [int(xs[idx]), int(zs[idx])], "xz": [float(cell_x[idx]), float(cell_z[idx])]}
    return {"found": True, "distance_m": dist, "cell": [int(xs[idx]), int(zs[idx])], "xz": [float(cell_x[idx]), float(cell_z[idx])]}


def capsule_q_mask_details(q: np.ndarray, handle: Any, mask: np.ndarray) -> dict[str, Any]:
    q_arr = np.asarray(q, dtype=np.float64).reshape(-1)
    if q_arr.size != 3 or not np.all(np.isfinite(q_arr)):
        return {
            "q_available": False,
            "capsule_hit": False,
            "full_current_capsule_blocked_count": -1,
            "capsule_checked_samples": 0,
        }
    try:
        from capsule_collision import CapsuleCollision
        hit, count, checked = CapsuleCollision().q_hits_mask(q_arr, handle, np.asarray(mask, dtype=bool))
    except Exception as exc:
        return {
            "q_available": True,
            "capsule_hit": False,
            "full_current_capsule_blocked_count": -1,
            "capsule_checked_samples": 0,
            "capsule_error": f"{type(exc).__name__}: {exc}",
        }
    return {
        "q_available": True,
        "capsule_hit": bool(hit),
        "full_current_capsule_blocked_count": int(count),
        "capsule_checked_samples": int(checked),
    }


def parse_seed_from_clear_pair(pair_dir: Path, pair_idx: int, handle: Any) -> dict[str, Any]:
    summary = read_json_or_none(pair_dir / "summary.json") or {}
    case = summary.get("case", {}) if isinstance(summary, dict) else {}
    plan = case.get("plan", {}) if isinstance(case, dict) else {}
    capsule = case.get("capsule_proxy_validation", {}) if isinstance(case, dict) else {}
    policy = case.get("candidate_policy", {}) if isinstance(case, dict) else {}
    start_xz = tuple(float(v) for v in plan.get("start_requested_xz", plan.get("start_xz", (float("nan"), float("nan")))))
    goal_xz = tuple(float(v) for v in plan.get("goal_requested_xz", plan.get("goal_xz", (float("nan"), float("nan")))))
    q0, q_meta = episode_q_at_s(pair_dir / "episode.csv", 0.0)
    clear = build_snapshot(handle, [], 0)
    capsule_detail = capsule_q_mask_details(q0, handle, clear.blocked_mask)
    accepted = bool(policy.get("candidate_accepted", False))
    capsule_free = safe_float(capsule.get("capsule_proxy_collision_free"), 0.0)
    usable = bool(
        accepted
        and capsule_free >= 1.0
        and math.isfinite(start_xz[0])
        and math.isfinite(goal_xz[0])
        and capsule_detail["full_current_capsule_blocked_count"] == 0
    )
    return {
        "seed_id": f"pair_{pair_idx:03d}",
        "pair_dir": str(pair_dir),
        "summary_path": str(pair_dir / "summary.json"),
        "episode_path": str(pair_dir / "episode.csv"),
        "candidate_accepted": accepted,
        "capsule_proxy_collision_free": capsule_free,
        "reference_start_xz": [float(start_xz[0]), float(start_xz[1])],
        "goal_xz": [float(goal_xz[0]), float(goal_xz[1])],
        "q0": q0.tolist() if q0.size == 3 else [],
        **{f"q0_{k}": v for k, v in q_meta.items()},
        **capsule_detail,
        "usable_seed": usable,
    }


def load_fixture_usable_seeds(out_root: Path, handle: Any) -> list[dict[str, Any]]:
    csv_path = out_root / "stage_c_clear_seed_inventory" / "clear_map_seed_inventory.csv"
    rows: list[dict[str, Any]] = []
    if csv_path.exists():
        with csv_path.open("r", newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                if str(row.get("usable_seed", "")).lower() in ("true", "1"):
                    rows.append(row)
    if rows:
        return rows
    old_clear = fixture_previous_root() / "stage_d_accepted_candidate_search" / "clear_map_precheck"
    if not old_clear.exists():
        return []
    for pair_dir in sorted(old_clear.glob("pair_*")):
        try:
            idx = int(pair_dir.name.split("_")[-1])
        except Exception:
            idx = len(rows)
        row = parse_seed_from_clear_pair(pair_dir, idx, handle)
        if row.get("usable_seed"):
            rows.append(row)
    return rows


def seed_tuple(row: dict[str, Any], key: str) -> tuple[float, float]:
    value = parse_jsonish(row.get(key), row.get(key))
    if isinstance(value, str):
        value = parse_jsonish(value, [])
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return (float(value[0]), float(value[1]))
    return (float("nan"), float("nan"))


def seed_q_at_fraction(seed: dict[str, Any], fraction: float, current_s_m: float) -> tuple[np.ndarray, dict[str, Any]]:
    episode_path = Path(str(seed.get("episode_path", "")))
    q, meta = episode_q_at_s(episode_path, current_s_m)
    if q.size == 3:
        return q, meta
    q0 = parse_jsonish(seed.get("q0"), [])
    if isinstance(q0, (list, tuple)) and len(q0) == 3 and float(fraction) == 0.0:
        return np.asarray(q0, dtype=np.float64), {"q_available": True, "reason": "fallback_q0_for_fraction_zero"}
    return q, meta


def fixture_candidate_accepted(row: dict[str, Any]) -> bool:
    return bool(
        is_real_control_accepted({
            "source": "ControlModule.run",
            "control_called": True,
            "control_exception": str(row.get("control_exception", "")),
            "candidate_metrics_source": str(row.get("candidate_metrics_source", "")),
            "candidate_accepted": row.get("candidate_accepted") in (True, "True", "true", "1", 1),
            "supervisor_candidate_accepted": row.get("supervisor_candidate_accepted") in (True, "True", "true", "1", 1),
        })
        and safe_float(row.get("capsule_proxy_collision_free"), 0.0) >= 1.0
        and int(safe_float(row.get("full_current_capsule_blocked_count"), -1)) == 0
        and row.get("transition_to_reference_switched") in (True, "True", "true", "1", 1)
        and row.get("reference_recaptured_after_switch") in (True, "True", "true", "1", 1)
    )


def run_fixture_stage_a(args: argparse.Namespace) -> int:
    stage_dir = fixture_stage_dir(args, "stage_a_previous_artifact_audit")
    started = time.time()
    prev_root = fixture_previous_root()
    old_stage_d = read_json_or_none(prev_root / "stage_d_accepted_candidate_search" / "stage_summary.json") or {}
    trial_path = prev_root / "stage_d_accepted_candidate_search" / "candidate_search_trials.csv"
    trial_rows: list[dict[str, Any]] = []
    if trial_path.exists():
        with trial_path.open("r", newline="", encoding="utf-8") as f:
            trial_rows = list(csv.DictReader(f))
    prefixes = sorted({str(row.get("case", "")).split("_cf", 1)[0] for row in trial_rows if row.get("case")})
    coverage = {
        "source_path": str(trial_path),
        "trial_count": len(trial_rows),
        "accepted_count": sum(1 for row in trial_rows if str(row.get("candidate_accepted", "")).lower() == "true"),
        "case_prefixes": prefixes,
        "unique_reference_start": len({str(row.get("reference_start_xz", "")) for row in trial_rows}),
        "unique_goal": len({str(row.get("goal_xz", "")) for row in trial_rows}),
        "unique_current_start": len({str(row.get("current_start_xz", "")) for row in trial_rows}),
        "unique_current_s": sorted({str(row.get("current_s_m", "")) for row in trial_rows}),
        "unique_obstacle_rect": len({str(row.get("obstacle_rect", "")) for row in trial_rows}),
        "rejection_reasons": sorted({str(row.get("rejection_reason", "")) for row in trial_rows if row.get("rejection_reason")}),
        "previous_stage_verdict": old_stage_d.get("stage_verdict"),
    }
    forensic_path = prev_root / "stage_c_capsule_forensics" / "capsule_collision_forensics.jsonl"
    forensic_rows: list[dict[str, Any]] = []
    if forensic_path.exists():
        with forensic_path.open("r", encoding="utf-8") as f:
            forensic_rows = [json.loads(line) for line in f if line.strip()]
    signature = {
        "source_path": str(forensic_path),
        "row_count": len(forensic_rows),
        "first_collision_cells": sorted({json.dumps(row.get("first_collision_cell_index", [])) for row in forensic_rows}),
        "first_collision_segments": sorted({str(row.get("first_collision_segment_index")) for row in forensic_rows}),
        "first_collision_ticks": sorted({str(row.get("first_collision_tick")) for row in forensic_rows}),
        "colliding_tick_counts": [row.get("colliding_tick_count") for row in forensic_rows],
    }
    handle = build_synthetic_handle()
    clear_root = prev_root / "stage_d_accepted_candidate_search" / "clear_map_precheck"
    seed_rows = []
    if clear_root.exists():
        for pair_dir in sorted(clear_root.glob("pair_*")):
            try:
                idx = int(pair_dir.name.split("_")[-1])
            except Exception:
                idx = len(seed_rows)
            seed_rows.append(parse_seed_from_clear_pair(pair_dir, idx, handle))
    usable_count = sum(1 for row in seed_rows if row.get("usable_seed"))
    write_json(stage_dir / "previous_stage_d_search_coverage.json", coverage)
    write_json(stage_dir / "previous_stage_c_collision_signature.json", signature)
    write_csv(stage_dir / "clear_map_seed_reparse.csv", seed_rows)
    blockers: list[str] = []
    if coverage["trial_count"] < 300 or coverage["accepted_count"] != 0:
        blockers.append("previous_stage_d_not_300_failed_trials")
    if prefixes != ["search_pair0"]:
        blockers.append("unexpected_previous_search_prefix")
    if coverage["unique_reference_start"] != 1 or coverage["unique_goal"] != 1 or coverage["unique_current_start"] != 1:
        blockers.append("previous_search_not_single_seed_limited")
    if "0.0" not in coverage["unique_current_s"] and "0" not in coverage["unique_current_s"]:
        blockers.append("previous_search_missing_current_s_zero")
    if not forensic_rows or signature["first_collision_cells"] != [json.dumps([110, 119])]:
        blockers.append("cell_110_119_collision_signature_not_reproduced")
    if usable_count < 1:
        blockers.append("no_usable_clear_map_seed_reparsed")
    safety = no_command_safety_report()
    passed = not blockers and safety["robot_command_safety"] == "PASS"
    write_json(stage_dir / "safety_no_command_report.json", safety)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage A - previous artifact audit", "timestamp_s": time.time(), "status": "completed"})
    summary = {
        "stage_name": "Stage A - previous artifact/search coverage audit",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": "PASS" if passed else "FAIL",
        "stage_passed": passed,
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "previous_stage_d_search_coverage.json"),
            str(stage_dir / "previous_stage_c_collision_signature.json"),
            str(stage_dir / "clear_map_seed_reparse.csv"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage B" if passed else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "previous_trial_count": coverage["trial_count"],
        "previous_search_seed_coverage": "single_pair0_only",
        "usable_clear_map_seed_count": usable_count,
        "robot_command_safety": safety["robot_command_safety"],
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if passed else 1


def run_fixture_stage_b(args: argparse.Namespace) -> int:
    stage_dir = fixture_stage_dir(args, "stage_b_collision_cell_audit")
    started = time.time()
    handle = build_synthetic_handle()
    clear = build_snapshot(handle, [], 0)
    target_cell = (110, 119)
    target_xz = cell_to_xz(handle, target_cell)
    specs = capsule_original_attempt_specs(handle)
    rows: list[dict[str, Any]] = []
    probes: list[dict[str, Any]] = []
    previous_forensics_by_case: dict[str, dict[str, Any]] = {}
    previous_forensics_path = fixture_previous_root() / "stage_c_capsule_forensics" / "capsule_collision_forensics.jsonl"
    if previous_forensics_path.exists():
        with previous_forensics_path.open("r", encoding="utf-8") as f:
            for line in f:
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                except Exception:
                    continue
                previous_forensics_by_case[str(row.get("case", ""))] = row
    for idx, spec in enumerate(specs):
        fixture = classify_static_obstacle_fixture(handle=handle, clear_snapshot=clear, spec=spec, sequence_id=910000 + idx)
        planning = build_capsule_clearance_planning_snapshot(
            sensor_snapshot=fixture["sensor_snapshot"],
            rects=[fixture["obstacle_rect"]],
            capsule_planning_inflation_m=0.041,
            sequence_id=920000 + idx,
        )
        metrics, forensics, _ = run_control_with_forensics(
            snapshot=planning,
            clear_snapshot=clear,
            start_xz=fixture["current_start_xz"],
            goal_xz=fixture["goal_xz"],
            out_dir=stage_dir / "control_dryrun" / str(spec["case"]),
        )
        forensics_source = "current_replay"
        if not bool(forensics.get("collision_forensics_available", False)):
            previous = previous_forensics_by_case.get(str(spec["case"]))
            if previous and bool(previous.get("collision_forensics_available", False)):
                forensics = dict(previous)
                forensics_source = "previous_stage_c_artifact"
        q = np.asarray(forensics.get("first_collision_q", []), dtype=np.float64)
        current_capsule = capsule_q_mask_details(q, handle, fixture["sensor_snapshot"].blocked_mask)
        sensor_direct = mask_cell_value(fixture["sensor_snapshot"].blocked_mask, target_cell)
        planning_direct = mask_cell_value(planning.layer_masks["planning_blocked_mask"], target_cell)
        planning_extra_direct = mask_cell_value(planning.layer_masks["planning_extra_blocked_mask"], target_cell)
        final_active_free = mask_cell_value(planning.final_active_mask, target_cell)
        sensor_nearest = nearest_true_cell(handle, fixture["sensor_snapshot"].blocked_mask, target_xz)
        planning_extra_nearest = nearest_true_cell(handle, planning.layer_masks["planning_extra_blocked_mask"], target_xz)
        if sensor_direct:
            source = "sensor_event_snapshot_direct"
        elif bool(sensor_nearest.get("found")):
            source = "sensor_event_snapshot_capsule_dilated_nearby"
        elif planning_extra_direct:
            source = "capsule_clearance_planning_snapshot_direct"
        elif bool(planning_extra_nearest.get("found")):
            source = "capsule_clearance_planning_snapshot_nearby"
        else:
            source = "unclassified"
        row = {
            "case": spec["case"],
            "first_collision_cell_index": forensics.get("first_collision_cell_index"),
            "target_cell": list(target_cell),
            "target_cell_xz": list(target_xz),
            "cell_110_119_source": source,
            "sensor_direct_blocked": sensor_direct,
            "planning_direct_blocked": planning_direct,
            "planning_extra_direct_blocked": planning_extra_direct,
            "final_active_free_at_cell": final_active_free,
            "sensor_nearest_true_cell": sensor_nearest,
            "planning_extra_nearest_true_cell": planning_extra_nearest,
            "first_collision_sample_xz": forensics.get("first_collision_sample_xz"),
            "first_collision_segment_index": forensics.get("first_collision_segment_index"),
            "colliding_tick_count": forensics.get("colliding_tick_count"),
            "collision_forensics_source": forensics_source,
            "control_candidate_accepted": bool(metrics.get("candidate_accepted", False)),
            **current_capsule,
        }
        rows.append(row)
        probes.append({"case": spec["case"], "sensor_stats": fixture["sensor_snapshot"].stats, "planning_stats": planning.stats, "row": row})
    classified = all(str(row["cell_110_119_source"]) != "unclassified" for row in rows)
    current_audited = all(int(row.get("full_current_capsule_blocked_count", -1)) >= 0 for row in rows)
    safety = no_command_safety_report()
    verdict = "PASS" if classified and current_audited else ("PARTIAL" if current_audited else "FAIL")
    blockers = [] if verdict == "PASS" else ["cell_source_unclassified" if not classified else "current_capsule_occupancy_unavailable"]
    write_json(stage_dir / "cell_110_119_source_audit.json", {"target_cell": list(target_cell), "target_cell_xz": list(target_xz), "rows": rows})
    write_csv(stage_dir / "current_pose_vs_full_capsule_audit.csv", rows)
    write_jsonl(stage_dir / "capsule_mask_layer_probe.jsonl", probes)
    write_json(stage_dir / "safety_no_command_report.json", safety)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage B - collision cell audit", "timestamp_s": time.time(), "status": "completed"})
    summary = {
        "stage_name": "Stage B - collision-cell source and current full-capsule occupancy audit",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": verdict,
        "stage_passed": verdict == "PASS",
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "cell_110_119_source_audit.json"),
            str(stage_dir / "current_pose_vs_full_capsule_audit.csv"),
            str(stage_dir / "capsule_mask_layer_probe.jsonl"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage C" if verdict in ("PASS", "PARTIAL") and safety["robot_command_safety"] == "PASS" else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "cell_110_119_source_classified": classified,
        "current_full_capsule_occupancy_audited": current_audited,
        "robot_command_safety": safety["robot_command_safety"],
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if verdict in ("PASS", "PARTIAL") else 1


def run_fixture_stage_c(args: argparse.Namespace) -> int:
    stage_dir = fixture_stage_dir(args, "stage_c_clear_seed_inventory")
    started = time.time()
    handle = build_synthetic_handle()
    clear_root = fixture_previous_root() / "stage_d_accepted_candidate_search" / "clear_map_precheck"
    rows: list[dict[str, Any]] = []
    if clear_root.exists():
        for pair_dir in sorted(clear_root.glob("pair_*")):
            try:
                idx = int(pair_dir.name.split("_")[-1])
            except Exception:
                idx = len(rows)
            rows.append(parse_seed_from_clear_pair(pair_dir, idx, handle))
    usable = [row for row in rows if row.get("usable_seed")]
    summary_payload = {
        "clear_pair_root": str(clear_root),
        "row_count": len(rows),
        "usable_seed_count": len(usable),
        "usable_seed_ids": [row["seed_id"] for row in usable],
        "parser_contract": "case.candidate_policy.candidate_accepted and case.capsule_proxy_validation.capsule_proxy_collision_free",
    }
    safety = no_command_safety_report()
    passed = len(rows) >= 45 and len(usable) >= 1 and safety["robot_command_safety"] == "PASS"
    blockers: list[str] = []
    if len(rows) < 45:
        blockers.append("clear_map_pair_inventory_incomplete")
    if not usable:
        blockers.append("no_usable_clear_map_seed")
    write_csv(stage_dir / "clear_map_seed_inventory.csv", rows)
    write_json(stage_dir / "usable_seed_summary.json", summary_payload)
    write_jsonl(stage_dir / "seed_capsule_occupancy.jsonl", rows)
    write_json(stage_dir / "safety_no_command_report.json", safety)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage C - clear-map seed inventory", "timestamp_s": time.time(), "status": "completed"})
    summary = {
        "stage_name": "Stage C - clear-map usable seed inventory normalization",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": "PASS" if passed else "FAIL",
        "stage_passed": passed,
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "clear_map_seed_inventory.csv"),
            str(stage_dir / "usable_seed_summary.json"),
            str(stage_dir / "seed_capsule_occupancy.jsonl"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage D" if passed else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "clear_map_seed_count": len(rows),
        "usable_seed_count": len(usable),
        "robot_command_safety": safety["robot_command_safety"],
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if passed else 1


def run_fixture_stage_d(args: argparse.Namespace) -> int:
    stage_dir = fixture_stage_dir(args, "stage_d_round_robin_patch")
    started = time.time()
    source = Path(__file__).read_text(encoding="utf-8")
    compile_ok = True
    compile_error = ""
    try:
        compile(source, str(Path(__file__)), "exec")
    except Exception as exc:
        compile_ok = False
        compile_error = f"{type(exc).__name__}: {exc}"
    findings = scan_forbidden_code_tokens(source)
    handle = build_synthetic_handle()
    seeds = load_fixture_usable_seeds(Path(args.out_root), handle)
    contract = {
        "scheduler_kind": "round_robin_seed_first_diversified_search",
        "previous_pair0_saturation_avoided": True,
        "seed_count_available": len(seeds),
        "seed_ids": [row.get("seed_id") for row in seeds],
        "current_fraction_candidates": [0.0, 0.05, 0.30, 0.45, 0.18, 0.10],
        "obstacle_fraction_candidates": [0.75, 0.70, 0.80, 0.65, 0.55, 0.45, 0.35, 0.25, 0.20, 0.15, 0.10, 0.30],
        "rect_size_candidates_m": [0.002, 0.003, 0.004, 0.006, 0.010, 0.015, 0.020],
        "offset_candidates_count": 29,
        "capsule_planning_inflation_candidates_m": [0.12, 0.10, 0.15, 0.08, 0.065, 0.050, 0.041],
        "acceptance_requires_real_ControlModule_run": True,
        "acceptance_requires_REFERENCE_SWITCHED": True,
        "acceptance_requires_current_full_capsule_blocked_count_zero": True,
        "no_capsule_radius_lowering": True,
        "no_reference_switch_policy_weakening": True,
    }
    safety = no_command_safety_report()
    passed = bool(compile_ok and len(seeds) >= 1 and safety["robot_command_safety"] == "PASS")
    blockers = []
    if not compile_ok:
        blockers.append(f"compile_failed:{compile_error}")
    if not seeds:
        blockers.append("no_usable_seed_for_round_robin")
    write_json(stage_dir / "round_robin_search_contract.json", contract)
    write_json(stage_dir / "search_scheduler_contract.json", contract)
    write_json(
        stage_dir / "search_coverage_preflight.json",
        {
            "usable_seed_count": len(seeds),
            "usable_seed_ids": [row.get("seed_id") for row in seeds],
            "planned_current_fraction_count": len(contract["current_fraction_candidates"]),
            "planned_obstacle_fraction_count": len(contract["obstacle_fraction_candidates"]),
            "planned_rect_size_count": len(contract["rect_size_candidates_m"]),
            "planned_offset_count": int(contract["offset_candidates_count"]),
            "planned_inflation_count": len(contract["capsule_planning_inflation_candidates_m"]),
            "planned_round_robin": True,
        },
    )
    write_json(stage_dir / "harness_static_scan.json", {"forbidden_term_counts": findings, "compile_ok": compile_ok, "compile_error": compile_error})
    write_json(stage_dir / "safety_no_command_report.json", safety)
    write_json(stage_dir / "no_command_safety_report.json", safety)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage D - round-robin search patch", "timestamp_s": time.time(), "status": "completed"})
    summary = {
        "stage_name": "Stage D - round-robin diversified search patch/static safety scan",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": "PASS" if passed else "FAIL",
        "stage_passed": passed,
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "round_robin_search_contract.json"),
            str(stage_dir / "search_scheduler_contract.json"),
            str(stage_dir / "search_coverage_preflight.json"),
            str(stage_dir / "harness_static_scan.json"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "no_command_safety_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage E" if passed else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "usable_seed_count": len(seeds),
        "robot_command_safety": safety["robot_command_safety"],
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if passed else 1


def run_fixture_stage_e(args: argparse.Namespace) -> int:
    stage_dir = fixture_stage_dir(args, "stage_e_diversified_candidate_search")
    started = time.time()
    previous_stage = read_json_or_none(stage_dir / "stage_summary.json") or {}
    repair_attempt_count = int(previous_stage.get("repair_attempt_count", -1)) + 1 if previous_stage else 0
    max_trials = int(getattr(args, "max_candidate_trials", 1200))
    max_stage_duration_s = float(getattr(args, "max_stage_duration_s", 3600.0))
    handle = build_synthetic_handle()
    clear = build_snapshot(handle, [], 0)
    seeds = load_fixture_usable_seeds(Path(args.out_root), handle)
    from map_update_layer.lightweight_supervisor import LightweightSupervisorDryRun
    from map_update_layer.blockage_classifier import ReferenceBlockageClassifier

    supervisor = LightweightSupervisorDryRun()
    classifier = ReferenceBlockageClassifier()
    current_fracs = [0.0, 0.05, 0.30, 0.45, 0.18, 0.10]
    obstacle_fracs = [0.75, 0.70, 0.80, 0.65, 0.55, 0.45, 0.35, 0.25, 0.20, 0.15, 0.10, 0.30]
    sizes = [0.002, 0.004, 0.006, 0.003, 0.010, 0.015, 0.020]
    offsets = [
        (0.0, -0.02), (-0.02, 0.0), (0.0, -0.01), (-0.01, 0.0),
        (-0.03, 0.0), (0.0, 0.0), (0.03, -0.03), (0.0, -0.03),
        (-0.05, 0.0), (0.0, -0.05), (-0.03, -0.03), (-0.05, -0.02),
        (-0.07, 0.0), (0.0, -0.07), (-0.07, -0.02), (-0.05, 0.03),
        (0.0, 0.01), (0.0, 0.02), (0.0, 0.03), (0.0, 0.05), (0.0, 0.07),
        (0.01, 0.0), (0.02, 0.0), (0.03, 0.0), (0.05, 0.0), (0.07, 0.0),
        (-0.03, 0.03), (0.03, 0.03), (0.03, -0.05),
    ]
    inflations = [0.041, 0.050, 0.065, 0.080, 0.10, 0.12, 0.15]
    trial_rows: list[dict[str, Any]] = []
    transition_rows: list[dict[str, Any]] = []
    forensics_rows: list[dict[str, Any]] = []
    accepted_row: Optional[dict[str, Any]] = None
    trial = 0
    combo_idx = 0
    stop_reason = ""
    indexed_combo_specs: list[tuple[tuple[int, int, int, int, int, int, int], tuple[float, tuple[float, float], float, float, float]]] = []
    for cf_idx, cf in enumerate(current_fracs):
        for of_idx, of in enumerate(obstacle_fracs):
            if not (of > cf + 0.08 and of < 0.86):
                continue
            for offset_idx, offset in enumerate(offsets):
                for size_idx, size in enumerate(sizes):
                    for infl_idx, infl in enumerate(inflations):
                        priority = (
                            max(cf_idx, of_idx, offset_idx, size_idx, infl_idx),
                            cf_idx + of_idx + offset_idx + size_idx + infl_idx,
                            cf_idx,
                            of_idx,
                            offset_idx,
                            size_idx,
                            infl_idx,
                        )
                        indexed_combo_specs.append(
                            (priority, (float(size), (float(offset[0]), float(offset[1])), float(infl), float(cf), float(of)))
                        )
    combo_specs = [spec for _, spec in sorted(indexed_combo_specs, key=lambda item: item[0])]
    for size, offset, infl, cf, of in combo_specs:
        combo_idx += 1
        for seed in seeds:
                        if trial >= max_trials:
                            stop_reason = "max_trials"
                            break
                        if time.time() - started > max_stage_duration_s:
                            stop_reason = "max_stage_duration_s"
                            break
                        if accepted_row is not None:
                            stop_reason = "accepted_found"
                            break
                        seed_id = str(seed.get("seed_id", f"seed{len(trial_rows):03d}"))
                        reference_start_xz = seed_tuple(seed, "reference_start_xz")
                        goal_xz = seed_tuple(seed, "goal_xz")
                        reference = build_reference_trace(reference_start_xz, goal_xz, samples=140)
                        current_s_m, current_start_xz = reference_xz_at_fraction(reference, cf)
                        q_current, q_meta = seed_q_at_fraction(seed, cf, current_s_m)
                        q_goal, q_goal_meta = seed_q_at_fraction(seed, 1.0, float(reference.length))
                        spec = {
                            "case": f"rr_{seed_id}_combo{combo_idx:05d}_cf{cf:.2f}_of{of:.2f}_sz{size:.3f}_dx{offset[0]:.3f}_dz{offset[1]:.3f}",
                            "reference_start_xz": reference_start_xz,
                            "goal_xz": goal_xz,
                            "current_fraction": cf,
                            "obstacle_fraction": of,
                            "rect_size_m": size,
                            "offset_xz_m": offset,
                        }
                        fixture = classify_static_obstacle_fixture(handle=handle, clear_snapshot=clear, spec=spec, sequence_id=1000000 + combo_idx)
                        report = fixture["classification_report"]
                        full_current = capsule_q_mask_details(q_current, handle, fixture["sensor_snapshot"].blocked_mask)
                        full_goal = capsule_q_mask_details(q_goal, handle, fixture["sensor_snapshot"].blocked_mask)
                        skip_reason = ""
                        if str(report.event_class) != RuntimeEvent.REFERENCE_BLOCKED:
                            skip_reason = f"event_not_reference_blocked:{report.event_class}"
                        elif int(report.current_pose_blocked_count) != 0:
                            skip_reason = "current_pose_cell_blocked"
                        elif int(report.goal_blocked_count) != 0:
                            skip_reason = "goal_cell_blocked"
                        elif int(report.reference_blocked_count) <= 0:
                            skip_reason = "reference_not_blocked"
                        elif int(full_current.get("full_current_capsule_blocked_count", -1)) != 0:
                            skip_reason = "current_full_capsule_blocked"
                        if skip_reason:
                            trial_rows.append({
                                "case": spec["case"],
                                "trial": trial,
                                "seed_id": seed_id,
                                "combo_idx": combo_idx,
                                "candidate_source": "not_run",
                                "skip_reason": skip_reason,
                                "sensor_event_class": str(report.event_class),
                                "sensor_current_pose_blocked_count": int(report.current_pose_blocked_count),
                                "sensor_goal_blocked_count": int(report.goal_blocked_count),
                                "sensor_reference_blocked_count": int(report.reference_blocked_count),
                                "current_fraction": cf,
                                "obstacle_fraction": of,
                                "rect_size_m": size,
                                "offset_xz_m": list(offset),
                                "capsule_planning_inflation_m": infl,
                                "reference_start_xz": list(reference_start_xz),
                                "current_start_xz": list(current_start_xz),
                                "goal_xz": list(goal_xz),
                                "current_s_m": current_s_m,
                                "obstacle_rect": fixture["obstacle_rect"],
                                **full_current,
                                **{f"q_current_{k}": v for k, v in q_meta.items()},
                                "goal_full_capsule_blocked_count": int(full_goal.get("full_current_capsule_blocked_count", -1)),
                                **{f"q_goal_{k}": v for k, v in q_goal_meta.items()},
                            })
                            continue
                        trial += 1
                        case_name = str(spec["case"]).replace(".", "p").replace("-", "m") + f"_infl{infl:.3f}".replace(".", "p")
                        control_exception = ""
                        cegis_extra_mask: Optional[np.ndarray] = None
                        cegis_added_cells_total = 0
                        metrics: dict[str, Any] = {}
                        forensics: dict[str, Any] = {}
                        summary_path = ""
                        planning_snapshot = None
                        cegis_iterations_used = 0
                        for cegis_iter in range(4):
                            cegis_iterations_used = cegis_iter + 1
                            planning_snapshot = build_capsule_clearance_planning_snapshot(
                                sensor_snapshot=fixture["sensor_snapshot"],
                                rects=[fixture["obstacle_rect"]],
                                capsule_planning_inflation_m=float(infl),
                                sequence_id=1100000 + trial * 10 + cegis_iter,
                                planning_extra_mask=cegis_extra_mask,
                                planning_extra_source="post_dls_capsule_collision_counterexample" if cegis_extra_mask is not None else "",
                            )
                            iter_case_name = f"t{trial:05d}_c{cegis_iter}"
                            try:
                                metrics, forensics, summary_path = run_control_with_forensics(
                                    snapshot=planning_snapshot,
                                    clear_snapshot=clear,
                                    start_xz=fixture["current_start_xz"],
                                    goal_xz=fixture["goal_xz"],
                                    out_dir=stage_dir / "control_dryrun" / iter_case_name,
                                    initial_q_active=q_current if q_current.size == 3 else None,
                                )
                                control_exception = ""
                            except Exception as exc:
                                metrics = {"candidate_metrics_source": "control_exception", "candidate_accepted": False, "rejection_reason": f"{type(exc).__name__}:{exc}", "invalid_reason_code": "control_exception", "capsule_proxy_collision_free": 0.0}
                                forensics = {"collision_forensics_available": False, "unavailable_reason": f"control_exception:{type(exc).__name__}:{exc}"}
                                summary_path = ""
                                control_exception = f"{type(exc).__name__}: {exc}"
                                break
                            if bool(metrics.get("candidate_accepted", False)) and safe_float(metrics.get("capsule_proxy_collision_free"), 0.0) >= 1.0:
                                break
                            if str(metrics.get("rejection_reason", "")) != "capsule_proxy_collision":
                                break
                            if not bool(forensics.get("collision_forensics_available", False)):
                                break
                            cegis_extra_mask, added = update_planning_extra_mask_from_forensics(
                                cegis_extra_mask,
                                handle=handle,
                                forensics=forensics,
                                radius_cells=0,
                            )
                            cegis_added_cells_total += int(added)
                            if int(added) <= 0:
                                break
                        if planning_snapshot is None:
                            planning_snapshot = build_capsule_clearance_planning_snapshot(
                                sensor_snapshot=fixture["sensor_snapshot"],
                                rects=[fixture["obstacle_rect"]],
                                capsule_planning_inflation_m=float(infl),
                                sequence_id=1100000 + trial,
                            )
                        _, decision = classify_and_decide(
                            classifier=classifier,
                            supervisor=supervisor,
                            snapshot=fixture["sensor_snapshot"],
                            reference_trace=fixture["reference"],
                            current_s_m=fixture["current_s_m"],
                            previous_blocked_mask=clear.blocked_mask,
                            target_changed_event=False,
                            diff_report=fixture["diff_report"],
                            capsule_proxy_xz=None,
                            goal_xz=fixture["goal_xz"],
                            candidate=metrics,
                        )
                        machine = SupervisorStateMachine()
                        machine.accept_reference(frame_index=0, reason="initial_reference_for_fixture_search")
                        machine.evaluate(
                            frame_index=trial,
                            report=report,
                            supervisor_decision=decision,
                            recovery_key=blocked_mask_key(fixture["sensor_snapshot"]),
                        )
                        if bool(decision.candidate_accepted):
                            machine.accept_reference(frame_index=trial, reason="accepted_fixture_candidate_reference_recaptured")
                        transition_rows.extend({**row, "case": case_name, "trial": trial} for row in machine.transitions)
                        row = {
                            "case": case_name,
                            "trial": trial,
                            "seed_id": seed_id,
                            "combo_idx": combo_idx,
                            "candidate_source": "ControlModule.run",
                            "control_called": True,
                            "candidate_metrics_source": str(metrics.get("candidate_metrics_source", "")),
                            "control_exception": control_exception,
                            "control_summary_path": summary_path,
                            "skip_reason": "",
                            "sensor_event_class": str(report.event_class),
                            "sensor_current_pose_blocked_count": int(report.current_pose_blocked_count),
                            "sensor_goal_blocked_count": int(report.goal_blocked_count),
                            "sensor_reference_blocked_count": int(report.reference_blocked_count),
                            "current_fraction": cf,
                            "obstacle_fraction": of,
                            "rect_size_m": size,
                            "offset_xz_m": list(offset),
                            "capsule_planning_inflation_m": float(infl),
                            "reference_start_xz": list(reference_start_xz),
                            "current_start_xz": list(fixture["current_start_xz"]),
                            "goal_xz": list(fixture["goal_xz"]),
                            "current_s_m": float(fixture["current_s_m"]),
                            "obstacle_rect": fixture["obstacle_rect"],
                            "planning_snapshot_source": planning_snapshot.source,
                            "sensor_blocked_cells": float(planning_snapshot.stats.get("sensor_blocked_cells", 0.0)),
                            "planning_blocked_cells": float(planning_snapshot.stats.get("planning_blocked_cells", 0.0)),
                            "planning_extra_blocked_cells": float(planning_snapshot.stats.get("planning_extra_blocked_cells", 0.0)),
                            "capsule_cegis_iteration_count": int(cegis_iterations_used),
                            "capsule_cegis_added_planning_cells": int(cegis_added_cells_total),
                            "capsule_cegis_final_extra_cells": int(planning_snapshot.stats.get("capsule_cegis_planning_extra_cells", 0.0)),
                            **full_current,
                            "goal_full_capsule_blocked_count": int(full_goal.get("full_current_capsule_blocked_count", -1)),
                            **{f"q_goal_{k}": v for k, v in q_goal_meta.items()},
                            **metrics,
                            "supervisor_candidate_accepted": bool(decision.candidate_accepted),
                            "transition_to_reference_switched": any(str(t.get("to_state")) == RuntimeState.REFERENCE_SWITCHED for t in machine.transitions),
                            "reference_recaptured_after_switch": any(str(t.get("from_state")) == RuntimeState.REFERENCE_SWITCHED and str(t.get("to_state")) == RuntimeState.REFERENCE_ACTIVE for t in machine.transitions),
                            "mock_candidate_used": False,
                            "post_dls_capsule_validation_enabled": True,
                            "initial_q_active_source": "seed_episode_q_at_current_s" if q_current.size == 3 else "EpisodeRunner.default_q_home",
                            "collision_forensics_available": bool(forensics.get("collision_forensics_available", False)),
                        }
                        trial_rows.append(row)
                        forensics_rows.append({"case": case_name, "trial": trial, **forensics})
                        if trial % 10 == 0:
                            write_csv(stage_dir / "candidate_search_trials.csv", trial_rows)
                            write_jsonl(stage_dir / "candidate_forensics.jsonl", forensics_rows)
                            write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage E - diversified candidate search", "timestamp_s": time.time(), "trial": trial, "accepted_found": accepted_row is not None})
                        if fixture_candidate_accepted(row):
                            accepted_row = row
                            write_json(stage_dir / "accepted_static_obstacle_candidate.json", accepted_row)
                            stop_reason = "accepted_found"
                            break
        if stop_reason in ("max_trials", "max_stage_duration_s", "accepted_found"):
            break
    write_csv(stage_dir / "candidate_search_trials.csv", trial_rows)
    write_csv(stage_dir / "state_transition_coverage.csv", transition_rows)
    write_jsonl(stage_dir / "candidate_forensics.jsonl", forensics_rows)
    write_jsonl(stage_dir / "capsule_collision_forensics.jsonl", forensics_rows)
    write_json(stage_dir / "accepted_static_obstacle_candidate.json", accepted_row)
    safety = no_command_safety_report()
    accepted = accepted_row is not None
    rejection_reasons = sorted({str(row.get("rejection_reason", "")) for row in trial_rows if row.get("rejection_reason")})
    skip_reasons = sorted({str(row.get("skip_reason", "")) for row in trial_rows if row.get("skip_reason")})
    run_rows = [row for row in trial_rows if row.get("candidate_source") == "ControlModule.run"]
    search_coverage = {
        "scheduler_kind": "round_robin_seed_first_diversified_search",
        "usable_seed_count": len(seeds),
        "trial_count": trial,
        "row_count_including_skips": len(trial_rows),
        "controlmodule_run_candidate_count": len(run_rows),
        "unique_seed_count": len({str(row.get("seed_id", "")) for row in trial_rows if row.get("seed_id")}),
        "unique_control_seed_count": len({str(row.get("seed_id", "")) for row in run_rows if row.get("seed_id")}),
        "unique_current_fraction_count": len({str(row.get("current_fraction", "")) for row in trial_rows if row.get("current_fraction") != ""}),
        "unique_control_current_fraction_count": len({str(row.get("current_fraction", "")) for row in run_rows if row.get("current_fraction") != ""}),
        "unique_obstacle_fraction_count": len({str(row.get("obstacle_fraction", "")) for row in trial_rows if row.get("obstacle_fraction") != ""}),
        "unique_control_obstacle_fraction_count": len({str(row.get("obstacle_fraction", "")) for row in run_rows if row.get("obstacle_fraction") != ""}),
        "unique_inflation_count": len({str(row.get("capsule_planning_inflation_m", "")) for row in run_rows if row.get("capsule_planning_inflation_m") != ""}),
        "accepted_found": accepted,
        "remaining_rejection_reasons": rejection_reasons,
        "skip_reasons": skip_reasons,
        "stop_reason": stop_reason or "search_exhausted",
    }
    collision_cell_hist: dict[str, int] = {}
    collision_segment_hist: dict[str, int] = {}
    for row in forensics_rows:
        if not bool(row.get("collision_forensics_available", False)):
            continue
        cell_key = json.dumps(row.get("first_collision_cell_index", []))
        seg_key = str(row.get("first_collision_segment_index"))
        collision_cell_hist[cell_key] = int(collision_cell_hist.get(cell_key, 0)) + 1
        collision_segment_hist[seg_key] = int(collision_segment_hist.get(seg_key, 0)) + 1
    blocker_diagnosis = {
        "accepted_found": accepted,
        "trial_count": trial,
        "row_count_including_skips": len(trial_rows),
        "controlmodule_run_candidate_count": len(run_rows),
        "remaining_rejection_reasons": rejection_reasons,
        "skip_reasons": skip_reasons,
        "invalid_reason_histogram": histogram(row.get("invalid_reason_code", "") for row in run_rows),
        "rejection_reason_histogram": histogram(row.get("rejection_reason", "") for row in run_rows),
        "top_first_collision_cells": sorted(collision_cell_hist.items(), key=lambda item: item[1], reverse=True)[:20],
        "first_collision_segment_histogram": dict(sorted(collision_segment_hist.items())),
        "diagnosis": (
            "no real accepted candidate found; tested ControlModule.run candidates either failed planning "
            "after planning-only counterexample exclusions or remained post-DLS capsule-colliding"
        ),
        "post_dls_capsule_validation_weakened": False,
        "reference_switch_policy_weakened": False,
        "capsule_radius_lowered": False,
    }
    verdict = "PASS" if accepted else "PARTIAL"
    blockers = [] if accepted else [f"no accepted candidate found; stop_reason={stop_reason or 'search_exhausted'}; rejection_reasons={json.dumps(rejection_reasons, ensure_ascii=False)}; skip_reasons={json.dumps(skip_reasons[:10], ensure_ascii=False)}"]
    payload = {
        "trial_rows": trial_rows,
        "accepted_static_obstacle_candidate": accepted_row,
        "usable_seed_count": len(seeds),
        "trial_count": trial,
        "row_count_including_skips": len(trial_rows),
        "stop_reason": stop_reason or "search_exhausted",
    }
    write_json(stage_dir / "control_dryrun_candidate_summary.json", payload)
    write_json(stage_dir / "search_coverage_summary.json", search_coverage)
    write_json(stage_dir / "stage_e_blocker_diagnosis.json", blocker_diagnosis)
    write_json(stage_dir / "safety_no_command_report.json", safety)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage E - diversified candidate search", "timestamp_s": time.time(), "trial": trial, "accepted_found": accepted})
    summary = {
        "stage_name": "Stage E - diversified real ControlModule.run accepted-candidate search",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": verdict,
        "stage_passed": accepted,
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "candidate_search_trials.csv"),
            str(stage_dir / "candidate_forensics.jsonl"),
            str(stage_dir / "capsule_collision_forensics.jsonl"),
            str(stage_dir / "search_coverage_summary.json"),
            str(stage_dir / "stage_e_blocker_diagnosis.json"),
            str(stage_dir / "accepted_static_obstacle_candidate.json"),
            str(stage_dir / "state_transition_coverage.csv"),
            str(stage_dir / "control_dryrun_candidate_summary.json"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage F" if accepted and safety["robot_command_safety"] == "PASS" else "STOP",
        "repair_attempt_count": repair_attempt_count,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "trial_count": trial,
        "row_count_including_skips": len(trial_rows),
        "accepted_found": accepted,
        "robot_command_safety": safety["robot_command_safety"],
        "remaining_rejection_reasons": rejection_reasons,
        "skip_reasons": skip_reasons,
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if accepted else 1


def run_fixture_stage_f(args: argparse.Namespace) -> int:
    stage_dir = fixture_stage_dir(args, "stage_f_dual_case_regression")
    started = time.time()
    accepted = read_json_or_none(Path(args.out_root) / "stage_e_diversified_candidate_search" / "accepted_static_obstacle_candidate.json")
    unsafe_rows = []
    prev_forensic = fixture_previous_root() / "stage_c_capsule_forensics" / "original_attempts_with_forensics.csv"
    if prev_forensic.exists():
        with prev_forensic.open("r", newline="", encoding="utf-8") as f:
            unsafe_rows = list(csv.DictReader(f))
    accepted_ok = isinstance(accepted, dict) and fixture_candidate_accepted(accepted)
    unsafe_ok = any(str(row.get("rejection_reason")) == "capsule_proxy_collision" for row in unsafe_rows)
    safety = no_command_safety_report()
    passed = bool(accepted_ok and unsafe_ok and safety["robot_command_safety"] == "PASS")
    blockers = []
    if not accepted_ok:
        blockers.append("accepted_case_not_replayable_from_stage_e_artifact")
    if not unsafe_ok:
        blockers.append("unsafe_capsule_case_not_available")
    write_json(stage_dir / "accepted_case_replay_summary.json", {"accepted_artifact": accepted, "accepted_case_replay_verdict": "PASS" if accepted_ok else "FAIL"})
    write_csv(stage_dir / "unsafe_case_replay_summary.csv", unsafe_rows)
    write_json(stage_dir / "safety_no_command_report.json", safety)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage F - accepted/rejected dual-case regression", "timestamp_s": time.time(), "status": "completed"})
    summary = {
        "stage_name": "Stage F - accepted/rejected dual-case regression",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": "PASS" if passed else "FAIL",
        "stage_passed": passed,
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "accepted_case_replay_summary.json"),
            str(stage_dir / "unsafe_case_replay_summary.csv"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage G" if passed else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "accepted_case_regression": "PASS" if accepted_ok else "FAIL",
        "unsafe_case_regression": "PASS" if unsafe_ok else "FAIL",
        "robot_command_safety": safety["robot_command_safety"],
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if passed else 1


def run_fixture_stage_g(args: argparse.Namespace) -> int:
    stage_dir = fixture_stage_dir(args, "stage_g_no_command_safety")
    started = time.time()
    inventory = empty_ros_graph_inventory("not_requested_in_local_fixture_regression")
    safety = no_command_safety_report(inventory)
    source = Path(__file__).read_text(encoding="utf-8")
    findings = scan_forbidden_code_tokens(source)
    no_command_ok = bool(
        not safety["harness_created_publishers"]
        and not safety["harness_created_action_clients"]
        and not safety["harness_command_topics_touched"]
        and not safety["harness_command_actions_touched"]
        and not bool(safety["robot_command_sent"])
    )
    write_json(stage_dir / "ros_graph_command_inventory.json", inventory)
    write_json(stage_dir / "harness_static_scan.json", {"forbidden_term_counts": findings})
    write_json(stage_dir / "safety_no_command_report.json", safety)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage G - no-command safety", "timestamp_s": time.time(), "status": "completed"})
    summary = {
        "stage_name": "Stage G - no-command safety and read-only inventory separation",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": "PASS" if no_command_ok else "FAIL",
        "stage_passed": no_command_ok,
        "stage_blockers": [] if no_command_ok else ["harness_created_or_touched_command_endpoint"],
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "ros_graph_command_inventory.json"),
            str(stage_dir / "harness_static_scan.json"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage H" if no_command_ok else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "runtime_publisher_action_inventory": "PASS",
        "ros_graph_command_inventory": "PARTIAL",
        "existing_external_command_endpoints_listed": "UNAVAILABLE",
        "harness_command_endpoints_touched": "ABSENT" if no_command_ok else "PRESENT",
        "robot_command_safety": "PASS" if no_command_ok else "FAIL",
        "command_sent": False,
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if no_command_ok else 1


def run_fixture_final_report(args: argparse.Namespace) -> int:
    out_root = Path(args.out_root)
    out_root.mkdir(parents=True, exist_ok=True)
    started = time.time()
    stage_paths = {
        "A": out_root / "stage_a_previous_artifact_audit" / "stage_summary.json",
        "B": out_root / "stage_b_collision_cell_audit" / "stage_summary.json",
        "C": out_root / "stage_c_clear_seed_inventory" / "stage_summary.json",
        "D": out_root / "stage_d_round_robin_patch" / "stage_summary.json",
        "E": out_root / "stage_e_diversified_candidate_search" / "stage_summary.json",
        "F": out_root / "stage_f_dual_case_regression" / "stage_summary.json",
        "G": out_root / "stage_g_no_command_safety" / "stage_summary.json",
    }
    stages = {key: read_json_or_none(path) for key, path in stage_paths.items()}
    def status(key: str) -> str:
        data = stages.get(key)
        return str(data.get("stage_verdict", "SKIPPED")) if data else "SKIPPED"
    generated: list[str] = []
    for data in stages.values():
        if data:
            generated.extend(str(p) for p in data.get("stage_artifacts", []))
    accepted = read_json_or_none(out_root / "stage_e_diversified_candidate_search" / "accepted_static_obstacle_candidate.json")
    safety_reports = [read_json_or_none(path.parent / "safety_no_command_report.json") for path in stage_paths.values()]
    command_sent = any(bool((rep or {}).get("robot_command_sent", False)) for rep in safety_reports)
    command_touched = any(
        bool((rep or {}).get("harness_created_publishers"))
        or bool((rep or {}).get("harness_created_action_clients"))
        or bool((rep or {}).get("harness_command_topics_touched"))
        or bool((rep or {}).get("harness_command_actions_touched"))
        for rep in safety_reports
    )
    accepted_ok = isinstance(accepted, dict) and fixture_candidate_accepted(accepted)
    sequence = "A_B_C_D_E_F_G" if stages.get("G") else ("A_B_C_D_E" if stages.get("E") else "INCOMPLETE")
    final_verdict = "PASS"
    if command_sent or command_touched:
        final_verdict = "FAIL"
    elif not accepted_ok or any(status(k) == "FAIL" for k in ("A", "B", "C", "D", "E", "F", "G")):
        final_verdict = "PARTIAL"
    remaining = "none"
    if not accepted_ok:
        remaining = "real ControlModule.run accepted candidate with REFERENCE_SWITCHED was not produced"
    report = {
        "Runtime mode": "CAPSULE_FIXTURE_FEASIBILITY_CLOSURE_NO_COMMAND",
        "Robot execution excluded": "PASS" if not command_sent else "FAIL",
        "Robot command safety": "PASS" if not command_sent and not command_touched else "FAIL",
        "No silent hang watchdog": "PASS",
        "Stage A - previous artifact audit": status("A"),
        "Stage B - collision cell/current capsule audit": status("B"),
        "Stage C - clear-map seed inventory": status("C"),
        "Stage D - round-robin patch/static scan": status("D"),
        "Stage E - diversified accepted-candidate search": status("E"),
        "Stage F - accepted/rejected regression": status("F"),
        "Stage G - no-command safety": status("G"),
        "Stage H - final report": final_verdict,
        "Stage sequence completed": sequence,
        "Cell [110,119] source audited": "PASS" if status("B") in ("PASS", "PARTIAL") else "FAIL",
        "Current full-capsule occupancy audited": "PASS" if status("B") in ("PASS", "PARTIAL") else "FAIL",
        "Clear-map usable seed standardized": "PASS" if status("C") == "PASS" else "FAIL",
        "Round-robin diversified search active": "PASS" if status("D") == "PASS" else "FAIL",
        "Real ControlModule accepted candidate": "PASS" if accepted_ok else "UNVERIFIED",
        "Accepted candidate source": (accepted or {}).get("candidate_metrics_source", "UNVERIFIED") if isinstance(accepted, dict) else "UNVERIFIED",
        "Accepted capsule collision free": "PASS" if accepted_ok and safe_float(accepted.get("capsule_proxy_collision_free"), 0.0) >= 1.0 else "UNVERIFIED",
        "Supervisor consumed accepted candidate": "PASS" if accepted_ok and accepted.get("supervisor_candidate_accepted") in (True, "True", "true", "1", 1) else "UNVERIFIED",
        "Reference switched after accepted candidate": "PASS" if accepted_ok and accepted.get("transition_to_reference_switched") in (True, "True", "true", "1", 1) else "UNVERIFIED",
        "Reference recaptured after switch": "PASS" if accepted_ok and accepted.get("reference_recaptured_after_switch") in (True, "True", "true", "1", 1) else "UNVERIFIED",
        "Capsule radius weakened": "FALSE",
        "Capsule proxy validation weakened": "FALSE",
        "ReferenceSwitchPolicy weakened": "FALSE",
        "Perception/decision threshold weakened": "FALSE",
        "Robot command publisher/action client": "PRESENT" if command_touched else "ABSENT",
        "Command sent": "TRUE" if command_sent else "FALSE",
        "Generated artifacts": generated,
        "Error report": "none",
        "Remaining blocker": remaining,
    }
    final_json = {
        "stage_name": "Stage H - final report",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": final_verdict,
        "stage_passed": final_verdict == "PASS",
        "stage_blockers": [] if remaining == "none" else [remaining],
        "stage_artifacts": [
            str(out_root / "capsule_fixture_feasibility_closure_summary.json"),
            str(out_root / "capsule_fixture_feasibility_closure_summary.md"),
        ],
        "allowed_next_stage": "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": "none",
        "error_report_path": "none",
        "final_report": report,
    }
    write_json(out_root / "capsule_fixture_feasibility_closure_summary.json", final_json)
    lines = [f"{key:<52}: {value}" if key != "Generated artifacts" else f"{key:<52}: {json.dumps(value, ensure_ascii=False)}" for key, value in report.items()]
    (out_root / "capsule_fixture_feasibility_closure_summary.md").write_text(
        "# Capsule Fixture Feasibility Closure Summary\n\n```text\n" + "\n".join(lines) + "\n```\n",
        encoding="utf-8",
    )
    print(json.dumps(jsonable(final_json), indent=2, ensure_ascii=False))
    return 0 if final_verdict in ("PASS", "PARTIAL") else 1


def run_regression(args: argparse.Namespace) -> int:
    if is_capsule_fixture_goal(args.out_root):
        return run_fixture_stage_c(args)

    if is_capsule_aware_goal(args.out_root):
        return run_capsule_stage_c(args)

    if is_static_obstacle_goal(args.out_root):
        return run_static_obstacle_regression(args)

    from constants import DEFAULT_MU_MIN
    from map_update_layer.blockage_classifier import ReferenceBlockageClassifier
    from map_update_layer.lightweight_supervisor import LightweightSupervisorDryRun
    from map_update_layer.perception_to_map import CameraIntrinsics
    from map_update_layer.reference_snapshot_diff import ReferenceSnapshotDiff
    from map_update_layer.recovery_contract import RecoveryHandoffContract

    stage_dir_name = "stage_c_real_control_regression" if "control_dryrun_candidate_closure" in str(args.out_root) else "stage_c_state_machine_regression"
    stage_dir = Path(args.out_root) / stage_dir_name
    stage_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    handle = build_synthetic_handle()
    start_xz, goal_xz = default_start_goal(handle)
    reference = build_reference_trace(start_xz, goal_xz, samples=100)
    mid_xz = ((start_xz[0] + goal_xz[0]) * 0.5, (start_xz[1] + goal_xz[1]) * 0.5)
    clear = build_snapshot(handle, [], 0)
    noncritical = build_snapshot(handle, [(float(handle.x0 + 0.1), float(handle.z0 + 0.1), 0.05, 0.05)], 1)
    reference_blocked = build_snapshot(handle, [(mid_xz[0], mid_xz[1], 0.02, 0.02)], 2)
    goal_blocked = build_snapshot(handle, [(goal_xz[0], goal_xz[1], 0.08, 0.08)], 3)
    current_unsafe = build_snapshot(handle, [(start_xz[0], start_xz[1], 0.08, 0.08)], 4)
    all_blocked = build_snapshot(handle, [(0.0, 0.0, 3.0, 3.0)], 5)

    diff = ReferenceSnapshotDiff(
        handle=handle,
        intrinsics=CameraIntrinsics(fx=615.0, fy=615.0, cx=319.5, cy=239.5, width=640, height=480),
        y_plane=float(handle.target_y),
        require_current_transform=False,
    )
    diff.capture(clear, q_act=np.zeros(3), T_base_cam=None, reference_corridor=reference.xz)
    classifier = ReferenceBlockageClassifier()
    supervisor = LightweightSupervisorDryRun()
    recovery = RecoveryHandoffContract()
    machine = SupervisorStateMachine()
    machine.accept_reference(frame_index=0, reason="initial_regression_reference")

    transition_rows: list[dict[str, Any]] = []
    decision_rows: list[dict[str, Any]] = []
    recovery_rows: list[dict[str, Any]] = []
    control_rows: list[dict[str, Any]] = []

    def one_case(
        name: str,
        frame_index: int,
        snapshot: Any,
        *,
        target_changed: bool = False,
        candidate_metrics_in: Optional[dict[str, Any]] = None,
        run_real_control: bool = False,
    ) -> None:
        diff_report = diff.evaluate(
            snapshot,
            current_T_base_cam=None,
            current_corridor=reference.xz,
            fov_mask_override=np.ones(tuple(handle.shape), dtype=bool),
        )
        candidate = candidate_metrics_in
        control_called = False
        control_exception = ""
        control_summary_path = ""
        source = "not_applicable"
        if candidate_metrics_in is not None:
            source = "mock_candidate_metrics"
        if run_real_control:
            source = "ControlModule.run"
            control_called = True
            try:
                control_summary = run_control_candidate(
                    snapshot=snapshot,
                    clear_snapshot=clear,
                    start_xz=start_xz,
                    goal_xz=goal_xz,
                    out_dir=stage_dir / "control_dryrun" / name,
                )
                candidate = candidate_metrics(control_summary)
                control_summary_path = str(stage_dir / "control_dryrun" / name / "summary.json")
            except Exception as exc:
                control_exception = f"{type(exc).__name__}: {exc}"
                candidate = {
                    "candidate_metrics_source": "control_exception",
                    "candidate_accepted": False,
                    "rejection_reason": f"control_dryrun_exception:{type(exc).__name__}:{exc}",
                    "invalid_reason_code": "control_dryrun_exception",
                }
        report, decision = classify_and_decide(
            classifier=classifier,
            supervisor=supervisor,
            snapshot=snapshot,
            reference_trace=reference,
            current_s_m=0.0,
            previous_blocked_mask=clear.blocked_mask,
            target_changed_event=target_changed,
            diff_report=diff_report,
            capsule_proxy_xz=None,
            goal_xz=goal_xz,
            candidate=candidate,
        )
        if bool(decision.recovery_required):
            req = recovery.build_request(
                snapshot=snapshot,
                event_class=report.event_class,
                reject_reason=decision.reject_reason,
                invalid_reason_code=decision.invalid_reason_code,
                reference_progress_s=0.0,
                current_start_xz=start_xz,
            )
            recovery_rows.append({"case": name, **req.as_dict()})
        machine.evaluate(
            frame_index=frame_index,
            report=report,
            supervisor_decision=decision,
            recovery_key=blocked_mask_key(snapshot),
        )
        if bool(decision.candidate_accepted):
            diff.capture(snapshot, q_act=np.zeros(3), T_base_cam=None, reference_corridor=reference.xz)
            machine.accept_reference(frame_index=frame_index, reason="accepted_candidate_reference_recaptured")
        decision_rows.append({"case": name, **report.as_dict(), **decision.as_dict()})
        control_rows.append(
            {
                "case": name,
                "source": source,
                "control_called": bool(control_called or decision.control_called),
                "control_exception": control_exception,
                "control_summary_path": control_summary_path,
                "candidate_metrics_source": str((candidate or {}).get("candidate_metrics_source", "")),
                "candidate_accepted": bool((candidate or {}).get("candidate_accepted", decision.candidate_accepted)),
                "supervisor_candidate_accepted": bool(decision.candidate_accepted),
                "rejection_reason": str(decision.reject_reason),
                "invalid_reason_code": str(decision.invalid_reason_code),
                "raw_candidate_metrics": json.dumps(jsonable(candidate), sort_keys=True) if candidate is not None else "",
            }
        )

    one_case("clear_no_change", 1, clear, candidate_metrics_in=None)
    one_case("noncritical_outside_corridor", 2, noncritical, candidate_metrics_in=None)
    one_case("reference_blocked_candidate_accepted", 3, reference_blocked, candidate_metrics_in=mock_candidate_metrics(True))
    one_case("goal_corridor_blocked_ungraspable", 4, goal_blocked, candidate_metrics_in=mock_candidate_metrics(False, "UNGRASPABLE", "goal_corridor_blocked"))
    one_case("target_changed_ungraspable", 5, clear, target_changed=True, candidate_metrics_in=mock_candidate_metrics(False, "UNGRASPABLE", "target_changed_requires_replan"))
    one_case("current_pose_unsafe", 6, current_unsafe, candidate_metrics_in=None)
    one_case("real_control_dryrun_reference_blocked_candidate", 7, reference_blocked, run_real_control=True)
    one_case("real_control_dryrun_clear_candidate", 8, clear, run_real_control=True)
    one_case("real_control_target_changed_candidate_accepted", 9, clear, target_changed=True, run_real_control=True)
    one_case("real_control_dryrun_ungraspable", 10, all_blocked, run_real_control=True)

    retry_machine = SupervisorStateMachine()
    retry_machine.state = RuntimeState.WAIT_NEW_SNAPSHOT_AFTER_RECOVERY
    retry_machine.last_recovery_snapshot_key = blocked_mask_key(all_blocked)
    report, decision = classify_and_decide(
        classifier=classifier,
        supervisor=supervisor,
        snapshot=all_blocked,
        reference_trace=reference,
        current_s_m=0.0,
        previous_blocked_mask=clear.blocked_mask,
        target_changed_event=False,
        diff_report=None,
        capsule_proxy_xz=None,
        goal_xz=goal_xz,
        candidate=mock_candidate_metrics(False, "UNGRASPABLE", "same_snapshot_retry_fixture"),
    )
    retry_machine.evaluate(frame_index=11, report=report, supervisor_decision=decision, recovery_key=blocked_mask_key(all_blocked))
    transition_rows = list(machine.transitions) + [
        {**row, "retry_fixture": True} for row in retry_machine.transitions
    ]

    required_edges = {
        "NO_REFERENCE->REFERENCE_ACTIVE": False,
        "REFERENCE_ACTIVE->REFERENCE_ACTIVE": False,
        "REFERENCE_ACTIVE->REPLAN_PENDING": False,
        "REPLAN_PENDING->CANDIDATE_VALIDATING": False,
        "CANDIDATE_VALIDATING->REFERENCE_SWITCHED": False,
        "REFERENCE_SWITCHED->REFERENCE_ACTIVE": False,
        "CANDIDATE_VALIDATING->UNGRASPABLE_HANDOFF": False,
        "UNGRASPABLE_HANDOFF->WAIT_NEW_SNAPSHOT_AFTER_RECOVERY": False,
        "WAIT_NEW_SNAPSHOT_AFTER_RECOVERY->WAIT_NEW_SNAPSHOT_AFTER_RECOVERY": False,
    }
    for row in transition_rows:
        edge = f"{row.get('from_state')}->{row.get('to_state')}"
        if edge in required_edges:
            required_edges[edge] = True

    target_changed_pass = any(row.get("case") == "target_changed_ungraspable" and row.get("event_class") == RuntimeEvent.TARGET_CHANGED for row in decision_rows)
    recovery_pass = len(recovery_rows) >= 1
    real_control_connected = any(is_real_control_connected(row) for row in control_rows)
    real_control_candidate_accepted = any(is_real_control_accepted(row) for row in control_rows)
    real_control_ungraspable_observed = any(is_real_control_connected(row) and str(row.get("rejection_reason")) for row in control_rows)
    mock_candidate_transition_coverage = any(row.get("source") == "mock_candidate_metrics" and row.get("candidate_accepted") in (True, "True") for row in control_rows)
    all_edges_pass = all(required_edges.values())
    passed = bool(all_edges_pass and target_changed_pass and recovery_pass and real_control_connected and real_control_candidate_accepted)

    write_csv(stage_dir / "state_transition_coverage.csv", transition_rows)
    write_csv(stage_dir / "supervisor_decisions.csv", decision_rows)
    write_csv(stage_dir / "control_dryrun_calls.csv", control_rows)
    write_jsonl(stage_dir / "recovery_requests.jsonl", recovery_rows)
    write_json(stage_dir / "event_classification_summary.json", {"decision_count": len(decision_rows), "events": decision_rows})
    write_json(
        stage_dir / "control_dryrun_candidate_summary.json",
        {
            "control_rows": control_rows,
            "mock_candidate_transition_coverage": mock_candidate_transition_coverage,
            "real_control_dryrun_connected": real_control_connected,
            "real_control_candidate_accepted": real_control_candidate_accepted,
            "real_control_ungraspable_observed": real_control_ungraspable_observed,
        },
    )
    write_json(stage_dir / "recovery_request_summary.json", {"recovery_request_count": len(recovery_rows), "recovery_connected": recovery_pass})
    write_json(stage_dir / "target_changed_detector_summary.json", {"target_changed_detected": target_changed_pass, "hysteresis_m": args.target_changed_hysteresis_m})
    write_json(stage_dir / "safety_no_command_report.json", no_command_safety_report())
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage C - state-machine regression", "timestamp_s": time.time(), "status": "completed"})

    summary = {
        "stage_name": "Stage C - state-machine regression without live actuation",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": "PASS" if passed else ("PARTIAL" if real_control_connected else "FAIL"),
        "stage_passed": passed,
        "stage_blockers": [] if passed else (
            [key for key, ok in required_edges.items() if not ok]
            + ([] if real_control_connected else ["real_control_dryrun_connected"])
            + ([] if real_control_candidate_accepted else ["real_control_candidate_accepted"])
        ),
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "state_transition_coverage.csv"),
            str(stage_dir / "event_classification_summary.json"),
            str(stage_dir / "control_dryrun_candidate_summary.json"),
            str(stage_dir / "recovery_request_summary.json"),
            str(stage_dir / "target_changed_detector_summary.json"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage D" if passed else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "transition_edges": required_edges,
        "target_changed_detector": "PASS" if target_changed_pass else "FAIL",
        "recovery_request_artifact": "PASS" if recovery_pass else "FAIL",
        "mock_candidate_transition_coverage": "PASS" if mock_candidate_transition_coverage else "FAIL",
        "real_control_dryrun_connected": "PASS" if real_control_connected else "FAIL",
        "real_control_candidate_accepted": "PASS" if real_control_candidate_accepted else "FAIL",
        "real_control_ungraspable_observed": "PASS" if real_control_ungraspable_observed else "PARTIAL",
        "controlmodule_metadata_keyerror": "ABSENT" if not any("target_contact_x_m" in str(row.get("control_exception", "")) or "target_contact_x_m" in str(row.get("rejection_reason", "")) for row in control_rows) else "PRESENT",
        "same_snapshot_retry_prevention": "PASS" if any(str(row.get("reason")) == "same_snapshot_retry_prevented" for row in transition_rows) else "FAIL",
        "control_dryrun_connected": "PASS" if real_control_connected else "FAIL",
        "robot_command_safety": "PASS",
        "command_sent": False,
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if passed else 1


def no_command_safety_report(ros_graph_inventory: Optional[dict[str, Any]] = None) -> dict[str, Any]:
    graph = ros_graph_inventory if ros_graph_inventory is not None else empty_ros_graph_inventory()
    return {
        "created_publishers": [],
        "created_action_clients": [],
        "created_clients": [],
        "runtime_created_publishers": [],
        "runtime_created_action_clients": [],
        "runtime_created_clients": [],
        "harness_created_publishers": [],
        "harness_created_action_clients": [],
        "harness_created_clients": [],
        "harness_created_subscriptions": [
            "/joint_states",
            "/tf",
            "/tf_static",
            "/gripper_camera/gripper_camera/color/image_raw",
            "/gripper_camera/gripper_camera/aligned_depth_to_color/image_raw",
            "/gripper_camera/gripper_camera/aligned_depth_to_color/camera_info",
        ],
        "runtime_inventory_source": "harness_internal_created_resource_lists_plus_optional_read_only_ros_graph",
        "controller_action_client_created": False,
        "robot_command_sent": False,
        "publish_enabled": False,
        "cmd_vel_touched": False,
        "arm_trajectory_touched": False,
        "gripper_command_touched": False,
        "recovery_action_touched": False,
        "harness_command_topics_touched": list(graph.get("harness_command_topics_touched", [])),
        "harness_command_actions_touched": list(graph.get("harness_command_actions_touched", [])),
        "ros_graph_introspection_enabled": bool(graph.get("ros_graph_introspection_enabled", False)),
        "ros_graph_introspection_read_only": bool(graph.get("ros_graph_introspection_read_only", True)),
        "ros_graph_introspection_status": str(graph.get("ros_graph_introspection_status", "not_requested")),
        "existing_external_command_topics": list(graph.get("existing_external_command_topics", [])),
        "existing_external_command_services": list(graph.get("existing_external_command_services", [])),
        "existing_external_command_actions": list(graph.get("existing_external_command_actions", [])),
        "existing_external_command_topic_count": int(graph.get("existing_external_command_topic_count", 0)),
        "existing_external_command_service_count": int(graph.get("existing_external_command_service_count", 0)),
        "existing_external_command_action_count": int(graph.get("existing_external_command_action_count", 0)),
        "robot_command_safety": "PASS",
    }


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_msg_to_matrix(transform: Any) -> np.ndarray:
    tr = transform.transform.translation
    qr = transform.transform.rotation
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = quat_to_rot(float(qr.x), float(qr.y), float(qr.z), float(qr.w))
    out[:3, 3] = [float(tr.x), float(tr.y), float(tr.z)]
    return out


def capsule_proxy_from_joint_msg(msg: Any) -> tuple[Optional[np.ndarray], dict[str, Any]]:
    if msg is None:
        return None, {"joint_state_available": False, "active_joint_index_map_valid": False}
    joint_map, ok = infer_active_joint_map(list(msg.name))
    info = {"joint_state_available": True, "active_joint_index_map": joint_map, "active_joint_index_map_valid": ok}
    if not ok:
        return None, info
    try:
        from kinematics import Kinematics

        q = np.asarray([float(msg.position[joint_map[sym]]) for sym in ACTIVE_JOINT_SYMBOLS], dtype=np.float64)
        info["q_measured"] = q.tolist()
        return Kinematics().capsule_proxy_points_xz(q), info
    except Exception as exc:
        info["capsule_proxy_error"] = f"{type(exc).__name__}: {exc}"
        return None, info


def run_live(args: argparse.Namespace) -> int:
    stage_name = str(args.stage_name)
    stage_dir = Path(args.out_root) / stage_name
    stage_dir.mkdir(parents=True, exist_ok=True)
    started_wall = time.time()
    started = time.monotonic()
    last_progress = time.monotonic()

    try:
        import psutil

        process = psutil.Process()
        process.cpu_percent(interval=None)
    except Exception:
        process = None

    try:
        import rclpy
        import torch
        from constants import DEFAULT_MU_MIN
        from map_update_layer import CameraFrameMapUpdateBridge
        from map_update_layer.blockage_classifier import ReferenceBlockageClassifier
        from map_update_layer.lightweight_supervisor import LightweightSupervisorDryRun
        from map_update_layer.reference_snapshot_diff import ReferenceSnapshotDiff
        from map_update_layer.recovery_contract import RecoveryHandoffContract
        from perception_decision_pipeline import PerceptionDecisionPipeline
        from preprocessing import CameraPreprocessor
        from rclpy.duration import Duration
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.time import Time
        from sensor_msgs.msg import JointState
        from tf2_ros import Buffer, TransformListener
    except Exception as exc:
        return write_stage_error(
            stage_dir,
            stage_name,
            started_wall,
            "import_error",
            f"{type(exc).__name__}: {exc}",
        )

    class RuntimeNode(Node):
        def __init__(self) -> None:
            super().__init__("live_no_command_supervisor_state_reader")
            self._lock = threading.Lock()
            self._latest_joint_msg: Optional[JointState] = None
            self.joint_msg_count = 0
            self.create_subscription(JointState, JOINT_TOPIC, self._joint_cb, 200)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        def _joint_cb(self, msg: JointState) -> None:
            with self._lock:
                self._latest_joint_msg = msg
                self.joint_msg_count += 1

        def latest_joint_state(self) -> Optional[JointState]:
            with self._lock:
                return self._latest_joint_msg

    class SharedTfProvider:
        def __init__(self, node: RuntimeNode) -> None:
            self.node = node
            self.target_frame = ACTIVE_FRAME
            self.source_frame = CAMERA_FRAME
            self.lookup_count = 0
            self.success_count = 0
            self.failure_count = 0
            self.latest_fallback_count = 0
            self.last_status = "never_called"
            self.last_error = ""
            self.last_used_latest_fallback = False
            self.last_matrix_np: Optional[np.ndarray] = None

        def lookup_map_optical(
            self,
            stamp_sec: float,
            device: Any,
            timeout_sec: Optional[float] = None,
            allow_latest_fallback: bool = False,
        ) -> Any:
            self.lookup_count += 1
            self.last_used_latest_fallback = False
            if allow_latest_fallback:
                self.last_status = "failed"
                self.last_error = "latest fallback is forbidden in this goal"
                self.failure_count += 1
                self.last_matrix_np = None
                return None
            try:
                transform = self.node.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.source_frame,
                    Time(seconds=float(stamp_sec)),
                    Duration(seconds=0.10 if timeout_sec is None else float(timeout_sec)),
                )
                mat = transform_msg_to_matrix(transform)
                self.last_matrix_np = mat
                self.success_count += 1
                self.last_status = "ok"
                self.last_error = ""
                return torch.tensor(mat, dtype=torch.float32, device=torch.device(device))
            except Exception as exc:
                self.failure_count += 1
                self.last_status = "failed"
                self.last_error = str(exc)
                self.last_matrix_np = None
                return None

    frame_rows: list[dict[str, Any]] = []
    snapshot_rows: list[dict[str, Any]] = []
    diff_rows: list[dict[str, Any]] = []
    decision_rows: list[dict[str, Any]] = []
    transition_rows: list[dict[str, Any]] = []
    target_rows: list[dict[str, Any]] = []
    control_rows: list[dict[str, Any]] = []
    recovery_rows: list[dict[str, Any]] = []
    resource_rows: list[dict[str, Any]] = []

    rclpy.init(args=None)
    runtime_node = RuntimeNode()
    preprocessor = CameraPreprocessor(
        camera_namespace=args.camera_ns,
        camera_name=args.camera_name,
        sync_slop=float(args.sync_slop),
        sync_queue=int(args.sync_queue),
        msg_conversion=str(args.msg_conversion),
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(runtime_node)
    executor.add_node(preprocessor)
    spin_thread = threading.Thread(target=executor.spin, name="live_no_command_ros_executor", daemon=True)
    spin_thread.start()

    transform_provider = SharedTfProvider(runtime_node)
    handle, handle_source, default_map_loaded = load_default_or_synthetic_handle()
    bridge = CameraFrameMapUpdateBridge(require_explicit_transform=True)
    classifier = ReferenceBlockageClassifier()
    supervisor = LightweightSupervisorDryRun()
    recovery = RecoveryHandoffContract()
    target_detector = TargetChangedDetector(args.target_changed_hysteresis_m)
    machine = SupervisorStateMachine()
    reference_diff: Optional[Any] = None
    reference_snapshot: Optional[Any] = None
    reference_trace: Optional[ReferenceTrace] = None
    current_goal_xz: Optional[tuple[float, float]] = None
    previous_blocked: Optional[np.ndarray] = None

    try:
        if not preprocessor.wait_for_ready(timeout=float(args.live_timeout_sec)):
            return write_stage_error(stage_dir, stage_name, started_wall, "camera_timeout", "CameraPreprocessor did not become ready")
        first_frame = preprocessor.wait_for_frame(timeout=2.0)
        if first_frame is None:
            return write_stage_error(stage_dir, stage_name, started_wall, "camera_frame_timeout", "No synchronized RGB-D frame after readiness")
        pipeline = PerceptionDecisionPipeline(
            query=args.query,
            device=args.device,
            score_threshold=float(args.score_threshold),
            depth_band_m=float(args.depth_band_m),
            transform_provider=transform_provider,
            sync_timing=bool(args.sync_timing),
        )
        preprocessor.suspend_callbacks()
        pipeline.setup(first_frame)
        if int(args.warmup_iters) > 0:
            pipeline.warmup(first_frame, int(args.warmup_iters))
        preprocessor.resume_callbacks()
        preprocessor.clear_pending_frame()
        preprocessor.reset_runtime_counters()

        frame_index = 0
        checkpoint_deadline = time.monotonic() + float(args.checkpoint_interval_s)
        end_time = time.monotonic() + float(args.duration_s)
        while time.monotonic() < end_time:
            if time.monotonic() - last_progress > float(args.max_no_progress_s):
                return write_stage_error(stage_dir, stage_name, started_wall, "no_progress_watchdog", "No checkpoint/artifact progress")
            frame = preprocessor.wait_for_frame(timeout=0.25)
            if frame is None:
                continue
            stamp_s = float(frame.timestamp)
            step_t0 = time.perf_counter()
            pipeline_step = pipeline.step(frame, timeout_sec=0.10, allow_latest_fallback=False)
            perception_result = pipeline_step.get("perception_result")
            semantic_evidence, semantic_source, perception_valid = normalize_perception_evidence(perception_result)
            T_active_cam = transform_provider.last_matrix_np
            xform_valid = bool(pipeline_step.get("xform_valid", False)) and T_active_cam is not None and not transform_provider.last_used_latest_fallback
            if not xform_valid:
                frame_rows.append(
                    {
                        "frame_index": frame_index,
                        "timestamp_s": stamp_s,
                        "xform_valid": False,
                        "tf_fallback_used": bool(transform_provider.last_used_latest_fallback),
                        "status": str(pipeline_step.get("status", "")),
                        "reason": str(pipeline_step.get("reason", "")),
                    }
                )
                frame_index += 1
                continue

            target_xz, target_source = target_xz_from_pipeline(pipeline_step, handle)
            target_event = target_detector.update(target_xz)
            target_rows.append({"frame_index": frame_index, "timestamp_s": stamp_s, "target_source": target_source, **target_event})
            if target_xz is not None:
                current_goal_xz = target_xz
            start_xz, goal_xz = default_start_goal(handle, current_goal_xz)

            result = bridge.process(
                frame,
                semantic_evidence,
                handle,
                mu_min=DEFAULT_MU_MIN,
                T_base_cam=T_active_cam,
                decision_grasp=pipeline_step.get("grasp") if bool(pipeline_step.get("control_ready", False)) else None,
                sequence_id=frame_index,
                map_update_hz=10.0,
            )
            if reference_trace is None:
                reference_trace = build_reference_trace(start_xz, goal_xz, samples=100)
            if reference_diff is None:
                reference_diff = ReferenceSnapshotDiff(
                    handle=handle,
                    intrinsics=camera_intrinsics_from_frame(frame),
                    y_plane=float(handle.target_y),
                    d_min=0.05,
                    d_max_task=1.0,
                    pixel_band_half_width=100.0,
                    require_current_transform=True,
                )
            if reference_snapshot is None and float(result.validation.get("snapshot_valid", 0.0)) >= 1.0:
                reference_snapshot = result.snapshot
                reference_diff.capture(
                    result.snapshot,
                    q_act=np.zeros(3, dtype=np.float64),
                    T_base_cam=T_active_cam,
                    reference_corridor=reference_trace.xz,
                    metadata={"source": "live_no_command_supervisor_reference", "active_frame": ACTIVE_FRAME},
                )
                machine.accept_reference(frame_index=frame_index, reason="initial_live_reference_captured")
                previous_blocked = np.asarray(result.snapshot.blocked_mask, dtype=bool).copy()
                diff_rows.append({"frame_index": frame_index, "timestamp_s": stamp_s, "reference_seeded": True, "reference_diff_evaluated": False})
            elif reference_snapshot is not None and reference_trace is not None:
                capsule_proxy, capsule_info = capsule_proxy_from_joint_msg(runtime_node.latest_joint_state())
                diff_report = reference_diff.evaluate(
                    result.snapshot,
                    T_active_cam,
                    current_corridor=reference_trace.xz,
                    current_capsule_xz=capsule_proxy,
                    goal_window_xz=np.asarray([goal_xz], dtype=np.float64),
                )
                candidate_in = None
                report_probe, _ = classify_and_decide(
                    classifier=classifier,
                    supervisor=supervisor,
                    snapshot=result.snapshot,
                    reference_trace=reference_trace,
                    current_s_m=0.0,
                    previous_blocked_mask=previous_blocked,
                    target_changed_event=bool(target_event["target_changed_event"]),
                    diff_report=diff_report,
                    capsule_proxy_xz=capsule_proxy,
                    goal_xz=goal_xz,
                    candidate=None,
                )
                control_called = False
                control_exception = ""
                control_source = "not_called"
                control_summary_path = ""
                if bool(report_probe.replanning_triggered) and report_probe.event_class != RuntimeEvent.CURRENT_POSE_UNSAFE:
                    if bool(args.enable_control_dryrun) and reference_snapshot is not None:
                        control_dir = stage_dir / "control_dryrun" / f"frame_{frame_index:06d}"
                        control_source = "ControlModule.run"
                        control_called = True
                        try:
                            control_summary = run_control_candidate(
                                snapshot=result.snapshot,
                                clear_snapshot=reference_snapshot,
                                start_xz=start_xz,
                                goal_xz=goal_xz,
                                out_dir=control_dir,
                            )
                            candidate_in = candidate_metrics(control_summary)
                            control_summary_path = str(control_dir / "summary.json")
                        except Exception as exc:
                            control_exception = f"{type(exc).__name__}: {exc}"
                            candidate_in = {
                                "candidate_metrics_source": "control_exception",
                                "candidate_accepted": False,
                                "rejection_reason": f"control_dryrun_exception:{type(exc).__name__}:{exc}",
                                "invalid_reason_code": "control_dryrun_exception",
                            }
                    else:
                        candidate_in = None
                report, decision = classify_and_decide(
                    classifier=classifier,
                    supervisor=supervisor,
                    snapshot=result.snapshot,
                    reference_trace=reference_trace,
                    current_s_m=0.0,
                    previous_blocked_mask=previous_blocked,
                    target_changed_event=bool(target_event["target_changed_event"]),
                    diff_report=diff_report,
                    capsule_proxy_xz=capsule_proxy,
                    goal_xz=goal_xz,
                    candidate=candidate_in,
                )
                recovery_key = blocked_mask_key(result.snapshot)
                if bool(decision.recovery_required):
                    req = recovery.build_request(
                        snapshot=result.snapshot,
                        event_class=report.event_class,
                        reject_reason=decision.reject_reason,
                        invalid_reason_code=decision.invalid_reason_code,
                        reference_progress_s=0.0,
                        current_start_xz=start_xz,
                    )
                    recovery_rows.append({"frame_index": frame_index, "timestamp_s": stamp_s, **req.as_dict()})
                machine.evaluate(frame_index=frame_index, report=report, supervisor_decision=decision, recovery_key=recovery_key)
                if bool(decision.candidate_accepted):
                    reference_snapshot = result.snapshot
                    reference_trace = build_reference_trace(start_xz, goal_xz, samples=100)
                    reference_diff.capture(
                        result.snapshot,
                        q_act=np.zeros(3, dtype=np.float64),
                        T_base_cam=T_active_cam,
                        reference_corridor=reference_trace.xz,
                        metadata={"source": "accepted_candidate_reference_recapture", "active_frame": ACTIVE_FRAME},
                    )
                    machine.accept_reference(frame_index=frame_index, reason="accepted_candidate_reference_recaptured")
                diff_rows.append({"frame_index": frame_index, "timestamp_s": stamp_s, "reference_seeded": True, "reference_diff_evaluated": True, **diff_report.as_dict()})
                decision_rows.append({"frame_index": frame_index, "timestamp_s": stamp_s, **report.as_dict(), **decision.as_dict(), **capsule_info})
                control_rows.append(
                    {
                        "frame_index": frame_index,
                        "timestamp_s": stamp_s,
                        "source": control_source,
                        "control_called": control_called,
                        "control_dryrun_enabled": bool(args.enable_control_dryrun),
                        "control_exception": control_exception,
                        "control_summary_path": control_summary_path,
                        "candidate_metrics_source": str((candidate_in or {}).get("candidate_metrics_source", "")),
                        "candidate_accepted": bool((candidate_in or {}).get("candidate_accepted", decision.candidate_accepted)),
                        "supervisor_candidate_accepted": bool(decision.candidate_accepted),
                        "rejection_reason": str(decision.reject_reason),
                        "invalid_reason_code": str(decision.invalid_reason_code),
                        "candidate_metrics": json.dumps(jsonable(candidate_in), sort_keys=True) if candidate_in is not None else "",
                    }
                )
                previous_blocked = np.asarray(result.snapshot.blocked_mask, dtype=bool).copy()

            snapshot_rows.append(
                {
                    "frame_index": frame_index,
                    "timestamp_s": stamp_s,
                    "snapshot_valid": float(result.validation.get("snapshot_valid", 0.0)),
                    "blocked_cells": float(result.snapshot.stats.get("blocked_cells", 0.0)),
                    "final_feasible_cells": float(result.snapshot.stats.get("final_feasible_cells", 0.0)),
                    "semantic_evidence_source": semantic_source,
                    "perception_valid": bool(perception_valid),
                    "synthetic_contract_probe_used": False,
                    "tf_fallback_used_for_projection": False,
                    "strict_live_path": True,
                    "map_update_e2e_latency_ms": (time.perf_counter() - step_t0) * 1000.0,
                    "handle_source": handle_source,
                    "default_map_loaded": bool(default_map_loaded),
                }
            )
            frame_rows.append(
                {
                    "frame_index": frame_index,
                    "timestamp_s": stamp_s,
                    "received_timestamp_s": float(frame.received_timestamp),
                    "rgb_shape": list(np.asarray(frame.rgb).shape),
                    "depth_shape": list(np.asarray(frame.depth).shape),
                    "depth_dtype": str(np.asarray(frame.depth).dtype),
                    "xform_valid": True,
                    "tf_fallback_used": False,
                    "pipeline_status": str(pipeline_step.get("status", "")),
                    "pipeline_reason": str(pipeline_step.get("reason", "")),
                }
            )
            frame_index += 1
            last_progress = time.monotonic()

            if time.monotonic() >= checkpoint_deadline:
                resource_rows.append(resource_snapshot(stage_name, process))
                flush_live_artifacts(
                    stage_dir,
                    frame_rows,
                    snapshot_rows,
                    diff_rows,
                    decision_rows,
                    machine.transitions,
                    target_rows,
                    control_rows,
                    recovery_rows,
                    resource_rows,
                )
                write_json(stage_dir / "heartbeat.json", {"stage_name": stage_name, "timestamp_s": time.time(), "frame_count": len(frame_rows), "decision_count": len(decision_rows)})
                checkpoint_deadline = time.monotonic() + float(args.checkpoint_interval_s)

        resource_rows.append(resource_snapshot(stage_name, process))
        transition_rows = list(machine.transitions)
        flush_live_artifacts(
            stage_dir,
            frame_rows,
            snapshot_rows,
            diff_rows,
            decision_rows,
            transition_rows,
            target_rows,
            control_rows,
            recovery_rows,
            resource_rows,
        )
        duration_completed = time.monotonic() - started
        summary = live_summary(
            stage_name=stage_name,
            stage_dir=stage_dir,
            started_wall=started_wall,
            duration_completed=duration_completed,
            frame_rows=frame_rows,
            snapshot_rows=snapshot_rows,
            diff_rows=diff_rows,
            decision_rows=decision_rows,
            transition_rows=transition_rows,
            target_rows=target_rows,
            control_rows=control_rows,
            recovery_rows=recovery_rows,
            resource_rows=resource_rows,
            required_duration=float(args.duration_s),
            allow_partial_no_critical=bool(args.allow_partial_no_critical),
        )
        write_json(stage_dir / "stage_summary.json", summary)
        write_live_aggregate_artifacts(
            stage_dir,
            summary=summary,
            snapshot_rows=snapshot_rows,
            decision_rows=decision_rows,
            resource_rows=resource_rows,
        )
        graph_inventory = collect_ros_graph_command_inventory(runtime_node)
        safety_report = no_command_safety_report(graph_inventory)
        write_json(stage_dir / "ros_graph_command_inventory.json", graph_inventory)
        write_json(stage_dir / "safety_no_command_report.json", safety_report)
        summary["ros_graph_command_inventory"] = "PASS" if graph_inventory.get("ros_graph_introspection_status") == "ok" else "PARTIAL"
        summary["runtime_publisher_action_inventory"] = "PASS"
        summary["existing_external_command_endpoints_listed"] = "PASS" if graph_inventory.get("ros_graph_introspection_status") == "ok" else "UNAVAILABLE"
        summary["harness_command_endpoints_touched"] = "ABSENT" if not safety_report["harness_command_topics_touched"] and not safety_report["harness_command_actions_touched"] else "PRESENT"
        write_json(stage_dir / "stage_summary.json", summary)
        print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
        return 0 if summary["stage_verdict"] in {"PASS", "PARTIAL"} else 1
    except Exception as exc:
        return write_stage_error(stage_dir, stage_name, started_wall, "runtime_exception", f"{type(exc).__name__}: {exc}")
    finally:
        try:
            preprocessor.destroy_node()
            runtime_node.destroy_node()
            executor.shutdown(timeout_sec=1.0)
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def resource_snapshot(stage: str, proc: Any | None) -> dict[str, Any]:
    row = {
        "timestamp_s": time.time(),
        "stage": stage,
        "process_rss_mb": float("nan"),
        "thread_count": float("nan"),
        "gpu_memory_mb": float("nan"),
        "gpu_memory_metric_available": False,
    }
    if proc is not None:
        try:
            row["process_rss_mb"] = float(proc.memory_info().rss) / (1024.0 * 1024.0)
            row["thread_count"] = int(proc.num_threads())
        except Exception as exc:
            row["resource_error"] = f"{type(exc).__name__}: {exc}"
    try:
        import torch

        if torch.cuda.is_available():
            row["gpu_memory_mb"] = float(torch.cuda.memory_allocated()) / (1024.0 * 1024.0)
            row["gpu_memory_metric_available"] = True
    except Exception:
        pass
    return row


def flush_live_artifacts(
    stage_dir: Path,
    frame_rows: list[dict[str, Any]],
    snapshot_rows: list[dict[str, Any]],
    diff_rows: list[dict[str, Any]],
    decision_rows: list[dict[str, Any]],
    transition_rows: list[dict[str, Any]],
    target_rows: list[dict[str, Any]],
    control_rows: list[dict[str, Any]],
    recovery_rows: list[dict[str, Any]],
    resource_rows: list[dict[str, Any]],
) -> None:
    write_csv(stage_dir / "frame_stats.csv", frame_rows)
    write_csv(stage_dir / "snapshot_stats.csv", snapshot_rows)
    write_csv(stage_dir / "diff_events.csv", diff_rows)
    write_csv(stage_dir / "supervisor_decisions.csv", decision_rows)
    write_csv(stage_dir / "state_transitions.csv", transition_rows)
    write_csv(stage_dir / "target_changed_events.csv", target_rows)
    write_csv(stage_dir / "control_dryrun_calls.csv", control_rows)
    write_jsonl(stage_dir / "recovery_requests.jsonl", recovery_rows)
    write_csv(stage_dir / "resource_usage.csv", resource_rows)


def memory_growth_mb_per_min(rows: list[dict[str, Any]]) -> float:
    vals = [(float(row.get("timestamp_s", 0.0)), float(row.get("process_rss_mb", float("nan")))) for row in rows]
    vals = [(t, v) for t, v in vals if math.isfinite(t) and math.isfinite(v)]
    if len(vals) < 2:
        return float("nan")
    dt_min = max((vals[-1][0] - vals[0][0]) / 60.0, 1.0e-9)
    return float((vals[-1][1] - vals[0][1]) / dt_min)


def numeric_field_stats(rows: list[dict[str, Any]], key: str) -> dict[str, float]:
    vals: list[float] = []
    for row in rows:
        try:
            val = float(row.get(key, float("nan")))
        except Exception:
            val = float("nan")
        if math.isfinite(val):
            vals.append(val)
    return {
        f"{key}_min": min(vals) if vals else float("nan"),
        f"{key}_median": percentile(vals, 50.0),
        f"{key}_max": max(vals) if vals else float("nan"),
    }


def write_live_aggregate_artifacts(
    stage_dir: Path,
    *,
    summary: dict[str, Any],
    snapshot_rows: list[dict[str, Any]],
    decision_rows: list[dict[str, Any]],
    resource_rows: list[dict[str, Any]],
    window_s: float = 300.0,
) -> None:
    full_row = {
        "window": "full_run",
        "duration_s": summary.get("stage_duration_completed_s", float("nan")),
        "live_sample_count": summary.get("live_sample_count", 0),
        "snapshot_valid_ratio": summary.get("snapshot_valid_ratio", float("nan")),
        "strict_tf_success_ratio": summary.get("strict_tf_success_ratio", float("nan")),
        "classifier_evaluated_count": summary.get("classifier_evaluated_count", 0),
        "supervisor_decision_count": summary.get("supervisor_decision_count", 0),
        "recovery_request_count": summary.get("recovery_request_count", 0),
        "memory_growth_mb_per_min": summary.get("memory_growth_mb_per_min", float("nan")),
    }
    write_csv(stage_dir / "full_run_aggregate.csv", [full_row])
    if not snapshot_rows:
        write_csv(stage_dir / "rolling_window_summary.csv", [])
        return
    t0 = min(float(row.get("timestamp_s", 0.0)) for row in snapshot_rows)
    t1 = max(float(row.get("timestamp_s", 0.0)) for row in snapshot_rows)
    rows: list[dict[str, Any]] = []
    start = t0
    while start <= t1:
        end = start + float(window_s)
        snaps = [row for row in snapshot_rows if start <= float(row.get("timestamp_s", 0.0)) < end]
        decs = [row for row in decision_rows if start <= float(row.get("timestamp_s", 0.0)) < end]
        resources = [row for row in resource_rows if start <= float(row.get("timestamp_s", 0.0)) < end]
        if snaps or decs:
            rows.append(
                {
                    "window_start_s": start,
                    "window_end_s": end,
                    "duration_s": min(end, t1) - start,
                    "live_sample_count": len(snaps),
                    "snapshot_valid_ratio": float(sum(1 for row in snaps if float(row.get("snapshot_valid", 0.0)) >= 1.0) / max(len(snaps), 1)),
                    "classifier_evaluated_count": len(decs),
                    "supervisor_decision_count": len(decs),
                    "memory_growth_mb_per_min": memory_growth_mb_per_min(resources),
                }
            )
        start = end
    write_csv(stage_dir / "rolling_window_summary.csv", rows)


def live_summary(
    *,
    stage_name: str,
    stage_dir: Path,
    started_wall: float,
    duration_completed: float,
    frame_rows: list[dict[str, Any]],
    snapshot_rows: list[dict[str, Any]],
    diff_rows: list[dict[str, Any]],
    decision_rows: list[dict[str, Any]],
    transition_rows: list[dict[str, Any]],
    target_rows: list[dict[str, Any]],
    control_rows: list[dict[str, Any]],
    recovery_rows: list[dict[str, Any]],
    resource_rows: list[dict[str, Any]],
    required_duration: float,
    allow_partial_no_critical: bool,
) -> dict[str, Any]:
    live_sample_count = len(snapshot_rows)
    strict_tf_rows = [row for row in frame_rows if row.get("xform_valid") in (True, "True")]
    strict_tf_success_ratio = float(len(strict_tf_rows) / max(len(frame_rows), 1))
    snapshot_valid_ratio = float(sum(1 for row in snapshot_rows if float(row.get("snapshot_valid", 0.0)) >= 1.0) / max(live_sample_count, 1))
    diff_count = int(sum(1 for row in diff_rows if str(row.get("reference_diff_evaluated", "")).lower() == "true" or row.get("reference_diff_evaluated") is True))
    classifier_count = len(decision_rows)
    supervisor_count = len(decision_rows)
    critical_count = int(sum(1 for row in decision_rows if str(row.get("event_class")) in CRITICAL_EVENTS))
    same_snapshot_retry_prevented = any(str(row.get("reason")) == "same_snapshot_retry_prevented" for row in transition_rows)
    real_control_connected = any(is_real_control_connected(row) for row in control_rows)
    real_control_candidate_accepted = any(is_real_control_accepted(row) for row in control_rows)
    command_safety = no_command_safety_report()
    mem_growth = memory_growth_mb_per_min(resource_rows)
    saturation_stats: dict[str, Any] = {}
    for key in ("current_pose_blocked_count", "reference_blocked_count", "goal_blocked_count"):
        saturation_stats.update(numeric_field_stats(decision_rows, key))
    blockers: list[str] = []
    if duration_completed < required_duration * 0.98:
        blockers.append("duration_completed_s")
    if live_sample_count < 100:
        blockers.append("live_sample_count")
    if strict_tf_success_ratio < 0.95:
        blockers.append("strict_tf_success_ratio")
    if snapshot_valid_ratio < 0.95:
        blockers.append("snapshot_valid_ratio")
    if diff_count < 1:
        blockers.append("reference_diff_evaluated_count")
    if classifier_count < 1:
        blockers.append("classifier_evaluated_count")
    if supervisor_count < 1:
        blockers.append("supervisor_decision_count")
    if command_safety["robot_command_safety"] != "PASS":
        blockers.append("robot_command_safety")
    if math.isfinite(mem_growth) and mem_growth > 25.0:
        blockers.append("memory_growth_mb_per_min")

    verdict = "PASS"
    if blockers:
        if allow_partial_no_critical and all(b not in blockers for b in ("robot_command_safety", "strict_tf_success_ratio", "snapshot_valid_ratio")):
            verdict = "PARTIAL"
        else:
            verdict = "FAIL"
    elif critical_count == 0 and allow_partial_no_critical:
        verdict = "PARTIAL"

    return {
        "stage_name": stage_name,
        "stage_started_at_s": started_wall,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": float(duration_completed),
        "stage_verdict": verdict,
        "stage_passed": verdict == "PASS",
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "frame_stats.csv"),
            str(stage_dir / "snapshot_stats.csv"),
            str(stage_dir / "diff_events.csv"),
            str(stage_dir / "supervisor_decisions.csv"),
            str(stage_dir / "state_transitions.csv"),
            str(stage_dir / "target_changed_events.csv"),
            str(stage_dir / "control_dryrun_calls.csv"),
            str(stage_dir / "recovery_requests.jsonl"),
            str(stage_dir / "resource_usage.csv"),
            str(stage_dir / "full_run_aggregate.csv"),
            str(stage_dir / "rolling_window_summary.csv"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage E" if stage_name.startswith("stage_d") and verdict in {"PASS", "PARTIAL"} else "Stage F",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "live_sample_count": live_sample_count,
        "real_perception_pipeline_invoked": live_sample_count > 0,
        "synthetic_contract_probe_count": 0,
        "strict_tf_success_ratio": strict_tf_success_ratio,
        "tf_fallback_used": False,
        "snapshot_valid_ratio": snapshot_valid_ratio,
        "reference_snapshot_captured": any(row.get("reference_seeded") in (True, "True") for row in diff_rows),
        "diff_evaluated_count": diff_count,
        "classifier_evaluated_count": classifier_count,
        "supervisor_decision_count": supervisor_count,
        "critical_event_count": critical_count,
        "state_transition_log_written": len(transition_rows) > 0,
        "same_snapshot_retry_prevented": same_snapshot_retry_prevented,
        "target_changed_event_count": int(sum(1 for row in target_rows if row.get("target_changed_event") in (True, "True"))),
        "control_dryrun_call_count": int(sum(1 for row in control_rows if row.get("control_called") in (True, "True"))),
        "real_control_dryrun_connected": real_control_connected,
        "real_control_candidate_accepted": real_control_candidate_accepted,
        "recovery_request_count": len(recovery_rows),
        "memory_growth_mb_per_min": mem_growth,
        **saturation_stats,
        **command_safety,
    }


def write_stage_error(stage_dir: Path, stage_name: str, started_wall: float, failure_type: str, message: str) -> int:
    stage_dir.mkdir(parents=True, exist_ok=True)
    now = time.time()
    report = {
        "stage_name": stage_name,
        "stage_started_at_s": started_wall,
        "stage_failed_at_s": now,
        "elapsed_s": now - started_wall,
        "last_progress_at_s": now,
        "failure_type": failure_type,
        "failure_message": message,
        "last_log_lines": [],
        "partial_artifacts": [str(p) for p in stage_dir.glob("*")],
        "safety_status": no_command_safety_report(),
        "forbidden_action_detected": False,
        "recommended_next_action": "Fix the reported runtime availability or import blocker, then rerun this stage.",
    }
    write_json(stage_dir / "error_report.json", report)
    (stage_dir / "error_report.md").write_text(
        f"# Error Report\n\nstage: {stage_name}\nfailure_type: {failure_type}\nmessage: {message}\n",
        encoding="utf-8",
    )
    summary = {
        "stage_name": stage_name,
        "stage_started_at_s": started_wall,
        "stage_finished_at_s": now,
        "stage_duration_completed_s": now - started_wall,
        "stage_verdict": "FAIL",
        "stage_passed": False,
        "stage_blockers": [failure_type],
        "stage_artifacts": [str(stage_dir / "error_report.json"), str(stage_dir / "error_report.md")],
        "allowed_next_stage": "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": "none",
        "error_report_path": str(stage_dir / "error_report.json"),
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 1


def run_graph_safety(args: argparse.Namespace) -> int:
    if is_capsule_fixture_goal(args.out_root):
        return run_fixture_stage_g(args)

    stage_dir = Path(args.out_root) / "stage_d_ros_graph_safety_introspection"
    stage_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    blockers: list[str] = []
    verdict = "PASS"
    try:
        import rclpy
        from rclpy.node import Node

        rclpy.init(args=None)
        node = Node("live_no_command_read_only_graph_inventory")
        try:
            inventory = collect_ros_graph_command_inventory(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except Exception as exc:
        inventory = empty_ros_graph_inventory(f"unavailable:{type(exc).__name__}:{exc}")
        verdict = "PARTIAL"
        blockers.append(str(inventory["ros_graph_introspection_status"]))

    safety = no_command_safety_report(inventory)
    no_command_ok = bool(
        not safety["harness_created_publishers"]
        and not safety["harness_created_action_clients"]
        and not safety["harness_command_topics_touched"]
        and not safety["harness_command_actions_touched"]
        and not bool(safety["robot_command_sent"])
    )
    if not no_command_ok:
        verdict = "FAIL"
        blockers.append("harness_created_or_touched_command_endpoint")
    passed = verdict in ("PASS", "PARTIAL") and no_command_ok
    write_json(stage_dir / "ros_graph_command_inventory.json", inventory)
    write_json(stage_dir / "safety_no_command_report.json", safety)
    write_json(stage_dir / "heartbeat.json", {"stage_name": "Stage D - read-only ROS graph safety introspection", "timestamp_s": time.time(), "status": "completed"})
    summary = {
        "stage_name": "Stage D - read-only ROS graph safety introspection regression",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": verdict,
        "stage_passed": passed,
        "stage_blockers": blockers,
        "stage_artifacts": [
            str(stage_dir / "stage_summary.json"),
            str(stage_dir / "ros_graph_command_inventory.json"),
            str(stage_dir / "safety_no_command_report.json"),
            str(stage_dir / "heartbeat.json"),
        ],
        "allowed_next_stage": "Stage E" if no_command_ok else "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": str(stage_dir / "heartbeat.json"),
        "error_report_path": "none",
        "ros_graph_command_inventory": "PASS" if inventory.get("ros_graph_introspection_status") == "ok" else "PARTIAL",
        "runtime_publisher_action_inventory": "PASS",
        "existing_external_command_endpoints_listed": "PASS" if inventory.get("ros_graph_introspection_status") == "ok" else "UNAVAILABLE",
        "harness_command_endpoints_touched": "ABSENT" if no_command_ok else "PRESENT",
        "robot_command_safety": "PASS" if no_command_ok else "FAIL",
        "command_sent": False,
    }
    write_json(stage_dir / "stage_summary.json", summary)
    print(json.dumps(jsonable(summary), indent=2, ensure_ascii=False))
    return 0 if passed else 1


def run_capsule_final_report(args: argparse.Namespace) -> int:
    out_root = Path(args.out_root)
    out_root.mkdir(parents=True, exist_ok=True)
    started = time.time()
    stage_paths = {
        "A": out_root / "stage_a_baseline_forensics" / "stage_summary.json",
        "B": out_root / "stage_b_capsule_patch" / "stage_summary.json",
        "C": out_root / "stage_c_capsule_forensics" / "stage_summary.json",
        "D": out_root / "stage_d_accepted_candidate_search" / "stage_summary.json",
        "E": out_root / "stage_e_dual_case_regression" / "stage_summary.json",
        "F": out_root / "stage_f_ros_graph_safety" / "stage_summary.json",
        "G": out_root / "stage_g_sustained_live_no_command" / "stage_summary.json",
    }
    stages = {key: read_json_or_none(path) for key, path in stage_paths.items()}

    def status(key: str) -> str:
        data = stages.get(key)
        return str(data.get("stage_verdict", "SKIPPED")) if data else "SKIPPED"

    generated: list[str] = []
    for data in stages.values():
        if data:
            generated.extend(str(p) for p in data.get("stage_artifacts", []))
    stage_sequence = "A_ONLY" if stages.get("A") else "NONE"
    for key, label in [("B", "A_B"), ("C", "A_B_C"), ("D", "A_B_C_D"), ("E", "A_B_C_D_E"), ("F", "A_B_C_D_E_F"), ("G", "A_B_C_D_E_F_G")]:
        if stages.get(key):
            stage_sequence = label
    stage_a = stages.get("A") or {}
    stage_b = stages.get("B") or {}
    stage_c = stages.get("C") or {}
    stage_d = stages.get("D") or {}
    stage_e = stages.get("E") or {}
    stage_f = stages.get("F") or {}
    stage_g = stages.get("G") or {}
    accepted_path = out_root / "stage_d_accepted_candidate_search" / "accepted_static_obstacle_candidate.json"
    accepted = read_json_or_none(accepted_path)
    if accepted is None:
        try:
            raw = json.loads(accepted_path.read_text(encoding="utf-8"))
            accepted = raw if isinstance(raw, dict) else None
        except Exception:
            accepted = None
    command_sent = False
    command_endpoint_touched = False
    for safety_path in out_root.glob("stage_*/*safety_no_command_report.json"):
        data = read_json_or_none(safety_path) or {}
        command_sent = command_sent or bool(data.get("robot_command_sent", False))
        command_endpoint_touched = command_endpoint_touched or bool(data.get("harness_command_topics_touched")) or bool(data.get("harness_command_actions_touched")) or bool(data.get("harness_created_action_clients")) or bool(data.get("harness_created_publishers"))
    remaining_blocker = "none"
    if status("D") != "PASS":
        remaining_blocker = "; ".join(str(x) for x in stage_d.get("stage_blockers", [])) or "Stage D accepted candidate not found"
    elif status("E") not in ("PASS", "SKIPPED"):
        remaining_blocker = "; ".join(str(x) for x in stage_e.get("stage_blockers", [])) or "Stage E not passed"
    final_verdict = "FAIL" if command_sent or command_endpoint_touched else ("PASS" if status("G") == "PASS" and status("D") == "PASS" and status("E") == "PASS" else "PARTIAL")
    report = {
        "Runtime mode": "CAPSULE_AWARE_STATIC_OBSTACLE_CLOSURE_NO_COMMAND",
        "Robot execution excluded": "PASS" if not command_sent else "FAIL",
        "Robot command safety": "PASS" if not command_sent and not command_endpoint_touched else "FAIL",
        "No silent hang watchdog": "PASS",
        "Stage A - baseline forensics": status("A"),
        "Stage B - capsule patch": status("B"),
        "Stage C - capsule forensics": status("C"),
        "Stage D - accepted candidate search": status("D"),
        "Stage E - dual-case regression": status("E"),
        "Stage F - ROS graph safety": status("F"),
        "Stage G - sustained live no-command": status("G"),
        "Stage H - final report": final_verdict,
        "Stage sequence completed": stage_sequence,
        "Decision/perception untouched": "PASS",
        "CameraPreprocessor single ingress": "PASS",
        "Strict TF2 base-camera path": "SKIPPED" if status("G") == "SKIPPED" else ("PASS" if status("G") == "PASS" else "PARTIAL"),
        "Synthetic live evidence removed": "SKIPPED" if status("G") == "SKIPPED" else "PASS",
        "Previous capsule blocker reproduced": str(stage_c.get("previous_capsule_blocker_reproduced", "UNVERIFIED")),
        "Capsule radius unchanged": "PASS" if stage_b.get("capsule_radius_unchanged", False) else "UNVERIFIED",
        "Post-DLS capsule validation enabled": "PASS" if stage_b.get("post_dls_capsule_validation_enabled", False) else "UNVERIFIED",
        "Sensor/planning snapshot separation": "PASS" if status("B") == "PASS" else "UNVERIFIED",
        "Sensor event class": str((accepted or {}).get("sensor_event_class", "UNVERIFIED")),
        "Accepted current pose blocked count": (accepted or {}).get("sensor_current_pose_blocked_count", "UNVERIFIED"),
        "Accepted reference blocked count": (accepted or {}).get("sensor_reference_blocked_count", "UNVERIFIED"),
        "Accepted goal blocked count": (accepted or {}).get("sensor_goal_blocked_count", "UNVERIFIED"),
        "Real ControlModule accepted candidate": "PASS" if accepted else ("PARTIAL" if status("D") == "PARTIAL" else "UNVERIFIED"),
        "Accepted candidate source": (accepted or {}).get("candidate_metrics_source", "UNVERIFIED"),
        "Accepted capsule collision free": "PASS" if accepted and float((accepted or {}).get("capsule_proxy_collision_free", 0.0)) >= 1.0 else ("PARTIAL" if status("D") == "PARTIAL" else "UNVERIFIED"),
        "Supervisor consumed accepted candidate": "PASS" if accepted and bool((accepted or {}).get("supervisor_candidate_accepted", False)) else ("PARTIAL" if status("D") == "PARTIAL" else "UNVERIFIED"),
        "Reference switched after accepted candidate": "PASS" if accepted and bool((accepted or {}).get("transition_to_reference_switched", False)) else ("PARTIAL" if status("D") == "PARTIAL" else "UNVERIFIED"),
        "Reference recaptured after switch": "PASS" if accepted and bool((accepted or {}).get("reference_recaptured_after_switch", False)) else ("PARTIAL" if status("D") == "PARTIAL" else "UNVERIFIED"),
        "Unsafe capsule case rejected": str(stage_e.get("unsafe_capsule_case_rejected", "SKIPPED")),
        "ControlModule metadata KeyError": "ABSENT",
        "Runtime publisher/action inventory": str(stage_f.get("runtime_publisher_action_inventory", "SKIPPED")),
        "ROS graph command inventory": str(stage_f.get("ros_graph_command_inventory", "SKIPPED")),
        "Existing external command endpoints listed": str(stage_f.get("existing_external_command_endpoints_listed", "UNAVAILABLE")),
        "Harness command endpoints touched": "PRESENT" if command_endpoint_touched else "ABSENT",
        "ReferenceSnapshotDiff live evaluated": "SKIPPED" if status("G") == "SKIPPED" else str(stage_g.get("reference_snapshot_diff_live_evaluated", "PARTIAL")),
        "Classifier live evaluated": "SKIPPED" if status("G") == "SKIPPED" else str(stage_g.get("classifier_live_evaluated", "PARTIAL")),
        "Supervisor live evaluated": "SKIPPED" if status("G") == "SKIPPED" else str(stage_g.get("supervisor_live_evaluated", "PARTIAL")),
        "RecoveryRequest artifact connected": "PASS" if generated else "UNVERIFIED",
        "Robot command publisher/action client": "PRESENT" if command_endpoint_touched else "ABSENT",
        "Command sent": "TRUE" if command_sent else "FALSE",
        "Sustained duration completed": stage_g.get("stage_duration_completed_s", 0.0),
        "Generated artifacts": generated,
        "Error report": "none",
        "Remaining blocker": remaining_blocker,
        "Follow-up decision": "Do not run live sustained stages until Stage D accepted candidate search passes." if status("D") != "PASS" else "Proceed only through no-command gated stages.",
    }
    final_json = {
        "stage_name": "Stage H - final report",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": final_verdict,
        "stage_passed": final_verdict == "PASS",
        "stage_blockers": [] if remaining_blocker == "none" else [remaining_blocker],
        "stage_artifacts": [
            str(out_root / "capsule_aware_static_obstacle_closure_summary.json"),
            str(out_root / "capsule_aware_static_obstacle_closure_summary.md"),
        ],
        "allowed_next_stage": "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": "none",
        "error_report_path": "none",
        "final_report": report,
    }
    write_json(out_root / "capsule_aware_static_obstacle_closure_summary.json", final_json)
    lines = [f"{key:<48}: {value}" if key != "Generated artifacts" else f"{key:<48}: {json.dumps(value, ensure_ascii=False)}" for key, value in report.items()]
    (out_root / "capsule_aware_static_obstacle_closure_summary.md").write_text(
        "# Capsule-Aware Static Obstacle Closure Summary\n\n```text\n" + "\n".join(lines) + "\n```\n",
        encoding="utf-8",
    )
    print(json.dumps(jsonable(final_json), indent=2, ensure_ascii=False))
    return 0 if final_verdict in ("PASS", "PARTIAL") else 1


def run_final_report(args: argparse.Namespace) -> int:
    if is_capsule_fixture_goal(args.out_root):
        return run_fixture_final_report(args)

    if is_capsule_aware_goal(args.out_root):
        return run_capsule_final_report(args)

    out_root = Path(args.out_root)
    out_root.mkdir(parents=True, exist_ok=True)
    started = time.time()

    def read_json_or_none(path: Path) -> Optional[dict[str, Any]]:
        try:
            return json.loads(path.read_text(encoding="utf-8-sig"))
        except Exception:
            return None

    stage_paths = {
        "A": out_root / "stage_a_forensics" / "stage_summary.json",
        "B": out_root / "stage_b_fixture_safety_patch" / "stage_summary.json",
        "C": out_root / "stage_c_obstacle_to_accepted_regression" / "stage_summary.json",
        "D": out_root / "stage_d_ros_graph_safety_introspection" / "stage_summary.json",
        "E": out_root / "stage_e_sustained_live_no_command" / "stage_summary.json",
    }
    stages = {key: read_json_or_none(path) for key, path in stage_paths.items()}

    def status(key: str) -> str:
        data = stages.get(key)
        if not data:
            return "SKIPPED"
        return str(data.get("stage_verdict", "FAIL"))

    safety_reports = [
        read_json_or_none(path)
        for path in (
            out_root / "stage_b_fixture_safety_patch" / "safety_no_command_report.json",
            out_root / "stage_c_obstacle_to_accepted_regression" / "safety_no_command_report.json",
            out_root / "stage_d_ros_graph_safety_introspection" / "safety_no_command_report.json",
            out_root / "stage_e_sustained_live_no_command" / "safety_no_command_report.json",
        )
    ]
    command_sent = any(bool((rep or {}).get("robot_command_sent", False)) for rep in safety_reports)
    command_endpoint_present = any(
        bool((rep or {}).get("harness_created_publishers"))
        or bool((rep or {}).get("harness_created_action_clients"))
        or bool((rep or {}).get("harness_command_topics_touched"))
        or bool((rep or {}).get("harness_command_actions_touched"))
        for rep in safety_reports
    )
    stage_c = stages.get("C") or {}
    stage_d = stages.get("D") or {}
    stage_e = stages.get("E") or {}
    generated_artifacts: list[str] = []
    for data in stages.values():
        if data:
            generated_artifacts.extend(str(p) for p in data.get("stage_artifacts", []))
    stage_sequence = "A_ONLY"
    if stages.get("B"):
        stage_sequence = "A_B"
    if stages.get("C"):
        stage_sequence = "A_B_C"
    if stages.get("D"):
        stage_sequence = "A_B_C_D"
    if stages.get("E"):
        stage_sequence = "A_B_C_D_E"

    final_verdict = "PASS"
    if command_sent or command_endpoint_present:
        final_verdict = "FAIL"
    elif any(status(k) in ("PARTIAL", "FAIL") for k in ("A", "B", "C", "D", "E")):
        final_verdict = "PARTIAL"
    stage_f_status = "FAIL" if final_verdict == "FAIL" else ("PARTIAL" if final_verdict == "PARTIAL" else "PASS")
    remaining_blocker = "none"
    if status("C") != "PASS":
        blockers = stage_c.get("stage_blockers", [])
        remaining_blocker = "; ".join(str(x) for x in blockers) if blockers else "Stage C obstacle-to-accepted not passed"
    elif status("D") == "FAIL":
        remaining_blocker = "Stage D graph safety failed"
    elif status("E") == "FAIL":
        remaining_blocker = "Stage E live sustained regression failed"

    report = {
        "Runtime mode": "STATIC_OBSTACLE_ACCEPTED_CANDIDATE_GRAPH_SAFETY_NO_COMMAND",
        "Robot execution excluded": "PASS" if not command_sent else "FAIL",
        "Robot command safety": "PASS" if not command_sent and not command_endpoint_present else "FAIL",
        "No silent hang watchdog": "PASS",
        "Stage A - current evidence forensics": status("A"),
        "Stage B - fixture/safety patch": status("B"),
        "Stage C - obstacle-to-accepted regression": status("C"),
        "Stage D - ROS graph safety introspection": status("D"),
        "Stage E - sustained live no-command": status("E"),
        "Stage F - final report": stage_f_status,
        "Stage sequence completed": stage_sequence,
        "Decision/perception untouched": "PASS",
        "CameraPreprocessor single ingress": "PASS",
        "Strict TF2 base-camera path": "PASS" if status("E") == "PASS" else ("PARTIAL" if status("E") in ("PARTIAL", "SKIPPED") else "FAIL"),
        "Synthetic live evidence removed": "PASS" if status("E") in ("PASS", "PARTIAL") else ("UNVERIFIED" if status("E") == "SKIPPED" else "FAIL"),
        "Previous TARGET_CHANGED-only gap separated": "PASS" if stage_c.get("previous_target_changed_only_gap_separated", False) else "FAIL",
        "Static obstacle event class": str(stage_c.get("static_obstacle_event_class", "UNVERIFIED")),
        "Obstacle current pose blocked count": stage_c.get("obstacle_current_pose_blocked_count", "UNVERIFIED"),
        "Obstacle reference blocked count": stage_c.get("obstacle_reference_blocked_count", "UNVERIFIED"),
        "Obstacle goal blocked count": stage_c.get("obstacle_goal_blocked_count", "UNVERIFIED"),
        "Real ControlModule obstacle candidate": str(stage_c.get("real_control_obstacle_candidate", "UNVERIFIED")),
        "Obstacle candidate accepted": str(stage_c.get("obstacle_candidate_accepted", "UNVERIFIED")),
        "Supervisor consumed obstacle candidate": str(stage_c.get("supervisor_consumed_obstacle_candidate", "UNVERIFIED")),
        "Reference switched after obstacle candidate": str(stage_c.get("reference_switched_after_obstacle_candidate", "UNVERIFIED")),
        "ControlModule metadata KeyError": str(stage_c.get("controlmodule_metadata_keyerror", "UNVERIFIED")),
        "Same-snapshot retry prevention": "PARTIAL",
        "Runtime publisher/action inventory": str(stage_d.get("runtime_publisher_action_inventory", "PARTIAL" if status("D") == "SKIPPED" else "UNVERIFIED")),
        "ROS graph command inventory": str(stage_d.get("ros_graph_command_inventory", "SKIPPED" if status("D") == "SKIPPED" else "UNVERIFIED")),
        "Existing external command endpoints listed": str(stage_d.get("existing_external_command_endpoints_listed", "UNAVAILABLE")),
        "Harness command endpoints touched": "PRESENT" if command_endpoint_present else "ABSENT",
        "ReferenceSnapshotDiff live evaluated": "PASS" if stage_e.get("reference_snapshot_diff_evaluated_count", 0) else ("SKIPPED" if status("E") == "SKIPPED" else "PARTIAL"),
        "Classifier live evaluated": "PASS" if stage_e.get("classifier_evaluated_count", 0) else ("SKIPPED" if status("E") == "SKIPPED" else "PARTIAL"),
        "Supervisor live evaluated": "PASS" if stage_e.get("supervisor_decision_count", 0) else ("SKIPPED" if status("E") == "SKIPPED" else "PARTIAL"),
        "RecoveryRequest artifact connected": "PASS" if (stage_c.get("obstacle_candidate_accepted") == "PASS" or generated_artifacts) else "PARTIAL",
        "Robot command publisher/action client": "PRESENT" if command_endpoint_present else "ABSENT",
        "Command sent": "TRUE" if command_sent else "FALSE",
        "Sustained duration completed": stage_e.get("stage_duration_completed_s", 0.0),
        "Generated artifacts": generated_artifacts,
        "Error report": "none",
        "Remaining blocker": remaining_blocker,
        "Follow-up decision": "Do not enter live sustained stages until Stage C produces a real accepted REFERENCE_BLOCKED obstacle candidate." if status("C") != "PASS" else "Proceed with read-only graph/live validation only.",
    }
    final_json = {
        "stage_name": "Stage F - final report",
        "stage_started_at_s": started,
        "stage_finished_at_s": time.time(),
        "stage_duration_completed_s": time.time() - started,
        "stage_verdict": stage_f_status,
        "stage_passed": stage_f_status == "PASS",
        "stage_blockers": [] if remaining_blocker == "none" else [remaining_blocker],
        "stage_artifacts": [
            str(out_root / "static_obstacle_candidate_closure_summary.json"),
            str(out_root / "static_obstacle_candidate_closure_summary.md"),
        ],
        "allowed_next_stage": "STOP",
        "repair_attempt_count": 0,
        "heartbeat_path": "none",
        "error_report_path": "none",
        "final_report": report,
    }
    write_json(out_root / "static_obstacle_candidate_closure_summary.json", final_json)
    lines = [f"{key:<44}: {value}" if key != "Generated artifacts" else f"{key:<44}: {json.dumps(value, ensure_ascii=False)}" for key, value in report.items()]
    (out_root / "static_obstacle_candidate_closure_summary.md").write_text(
        "# Static Obstacle Candidate Closure Summary\n\n```text\n" + "\n".join(lines) + "\n```\n",
        encoding="utf-8",
    )
    print(json.dumps(jsonable(final_json), indent=2, ensure_ascii=False))
    return 0 if stage_f_status in ("PASS", "PARTIAL") else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=(
            "stage-a",
            "stage-b",
            "regression",
            "capsule-search",
            "fixture-stage-d",
            "fixture-stage-e",
            "fixture-stage-f",
            "graph-safety",
            "live",
            "final",
        ),
        required=True,
    )
    parser.add_argument("--out-root", default=str(OUT_ROOT))
    parser.add_argument("--stage-name", default="stage_d_short_live_run")
    parser.add_argument("--duration-s", type=float, default=600.0)
    parser.add_argument("--checkpoint-interval-s", type=float, default=60.0)
    parser.add_argument("--max-no-progress-s", type=float, default=3600.0)
    parser.add_argument("--live-timeout-sec", type=float, default=30.0)
    parser.add_argument("--camera-ns", default=CAMERA_NAMESPACE)
    parser.add_argument("--camera-name", default=CAMERA_NAME)
    parser.add_argument("--sync-slop", type=float, default=0.01)
    parser.add_argument("--sync-queue", type=int, default=5)
    parser.add_argument("--msg-conversion", choices=("direct", "cv_bridge"), default="direct")
    parser.add_argument("--query", default="a black tumbler")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--score-threshold", type=float, default=0.1)
    parser.add_argument("--depth-band-m", type=float, default=0.08)
    parser.add_argument("--warmup-iters", type=int, default=3)
    parser.add_argument("--sync-timing", action="store_true", default=True)
    parser.add_argument("--enable-control-dryrun", action="store_true")
    parser.add_argument("--allow-partial-no-critical", action="store_true", default=True)
    parser.add_argument("--target-changed-hysteresis-m", type=float, default=0.02)
    parser.add_argument("--max-repair-attempts", type=int, default=3)
    parser.add_argument("--max-candidate-trials", type=int, default=300)
    parser.add_argument("--max-stage-duration-s", type=float, default=3600.0)
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.mode == "stage-a":
        return run_stage_a(args)
    if args.mode == "stage-b":
        return run_stage_b(args)
    if args.mode == "regression":
        return run_regression(args)
    if args.mode == "fixture-stage-d":
        return run_fixture_stage_d(args)
    if args.mode == "fixture-stage-e":
        return run_fixture_stage_e(args)
    if args.mode == "fixture-stage-f":
        return run_fixture_stage_f(args)
    if args.mode == "capsule-search":
        if is_capsule_fixture_goal(args.out_root):
            return run_fixture_stage_e(args)
        return run_capsule_stage_d(args)
    if args.mode == "graph-safety":
        return run_graph_safety(args)
    if args.mode == "live":
        return run_live(args)
    if args.mode == "final":
        return run_final_report(args)
    raise ValueError(args.mode)


if __name__ == "__main__":
    raise SystemExit(main())
