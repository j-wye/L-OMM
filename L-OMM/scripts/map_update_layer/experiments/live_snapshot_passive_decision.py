#!/usr/bin/env python3
"""Read-only live/saved snapshot feed into ReferenceBlockageClassifier.

This experiment creates one fixed accepted reference, then classifies
CameraFrameMapUpdateBridge snapshots without issuing robot commands and without
calling ControlModule.run inside the frame loop.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import time
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))

try:
    from control_module.capsule_collision import CapsuleCollision
    from control_module.constants import DEFAULT_COST_MODE, DEFAULT_MU_MIN
    from control_module.control_module import ControlModule
    from control_module.path_planning import PathPlanning
except (ImportError, ModuleNotFoundError):
    from capsule_collision import CapsuleCollision
    from constants import DEFAULT_COST_MODE, DEFAULT_MU_MIN
    from control_module import ControlModule
    from path_planning import PathPlanning

from map_update_layer.blockage_classifier import ReferenceBlockageClassifier
from map_update_layer.camera_frame_bridge import (
    CameraFrameMapUpdateBridge,
    apply_speed_prefilter,
    frame_to_adapter_frame,
    speed_prefilter_unknown_candidate_mask,
)
from map_update_layer.fov_mask import compute_task_plane_fov_mask
from map_update_layer.map_update_layer import MapUpdateLayer
from map_update_layer.map_update_request import MapUpdateRequest
from map_update_layer.path_collision_monitor import PathCollisionMonitor
from map_update_layer.perception_to_map import Y_PLANE_FIXED, nominal_t_base_cam

from control_passive_map_update_integration import _load_reference, _run_control


RUNTIME_MODE = "LIVE_GOAL_CORRIDOR_LAYER_EXPORT_HARDENING_NO_COMMAND"


def run_passive_decision(
    *,
    out: str,
    frames: int,
    use_saved_frame: Optional[str] = None,
    map_path: Optional[str] = None,
    camera_ns: str = "gripper_camera",
    camera_name: str = "gripper_camera",
    sync_slop: float = 0.03,
    sync_queue: int = 10,
    msg_conversion: str = "auto",
    live_timeout_sec: float = 12.0,
) -> Dict[str, Any]:
    out_root = Path(out)
    out_root.mkdir(parents=True, exist_ok=True)
    (out_root / "frames").mkdir(parents=True, exist_ok=True)

    planner = PathPlanning(map_path=map_path)
    handle = planner.load_map(map_path)
    updater = MapUpdateLayer()
    initial_snapshot = updater.build(
        handle,
        mu_min=DEFAULT_MU_MIN,
        request=MapUpdateRequest(source="live_snapshot_passive_initial_clear", sequence_id=1),
    )
    control = ControlModule(output_root=str(out_root / "initial_control"))
    control_summary = _run_control(
        control,
        active_snapshot=initial_snapshot,
        clear_snapshot=initial_snapshot,
        scenario="clear",
        out_dir=out_root / "initial_control",
        cost_mode=DEFAULT_COST_MODE,
        reference_backend="c2_quintic",
    )
    reference = _load_reference(out_root / "initial_control" / "reference.csv")
    q_traj = _load_q_traj(out_root / "initial_control" / "episode.csv")
    accepted_reference: Dict[str, Any] = {"xz_path": reference["xz"]}
    if q_traj.shape[0] == reference["xz"].shape[0] and q_traj.shape[0] > 0:
        accepted_reference["q_traj"] = q_traj
    np.savez(
        out_root / "accepted_reference.npz",
        s=reference["s"],
        xz=reference["xz"],
        q_traj=q_traj,
        initial_blocked_mask=np.asarray(initial_snapshot.blocked_mask, dtype=bool),
        initial_final_active_mask=np.asarray(initial_snapshot.final_active_mask, dtype=bool),
    )

    bridge = CameraFrameMapUpdateBridge()
    classifier = ReferenceBlockageClassifier(
        path_collision_monitor=PathCollisionMonitor(CapsuleCollision(), handle=handle)
    )
    T_base_cam = nominal_t_base_cam(Y_PLANE_FIXED)
    T_base_cam[:3, 3] = np.array([0.0, Y_PLANE_FIXED, 0.50], dtype=np.float64)

    rows: List[Dict[str, Any]]
    mode: str
    if use_saved_frame:
        mode = "saved_frame"
        rows = _run_saved_frames(
            bridge=bridge,
            classifier=classifier,
            handle=handle,
            accepted_reference=accepted_reference,
            reference=reference,
            initial_blocked_mask=initial_snapshot.blocked_mask,
            frame=_load_frame_npz(Path(use_saved_frame)),
            frame_count=int(frames),
            out_root=out_root,
            T_base_cam=T_base_cam,
        )
    else:
        mode, rows = _run_live_frames(
            bridge=bridge,
            classifier=classifier,
            handle=handle,
            accepted_reference=accepted_reference,
            reference=reference,
            initial_blocked_mask=initial_snapshot.blocked_mask,
            frame_count=int(frames),
            out_root=out_root,
            T_base_cam=T_base_cam,
            camera_ns=camera_ns,
            camera_name=camera_name,
            sync_slop=sync_slop,
            sync_queue=sync_queue,
            msg_conversion=msg_conversion,
            live_timeout_sec=live_timeout_sec,
        )

    _write_csv(out_root / "events.csv", rows)
    summary = {
        "runtime_mode": RUNTIME_MODE,
        "mode": mode,
        "control_module_called_before_loop": True,
        "control_module_called_in_live_loop": False,
        "accepted_reference_samples": int(reference["xz"].shape[0]),
        "accepted_reference_q_traj_samples": int(q_traj.shape[0]),
        "frame_requested_count": int(frames),
        "frame_processed_count": int(len(rows)),
        "classifier_invoked_per_frame": bool(len(rows) == int(frames)),
        "event_counts": _event_counts(rows),
        "reference_blocked_source_counts": _source_counts(rows),
        "robot_command_endpoint_touched": False,
        "control_summary_reject_reason": _nested_get(control_summary, ("case", "candidate_policy", "reject_reason"), ""),
    }
    _write_json(out_root / "summary.json", summary)
    return summary


def _run_saved_frames(
    *,
    bridge: CameraFrameMapUpdateBridge,
    classifier: ReferenceBlockageClassifier,
    handle: Any,
    accepted_reference: Mapping[str, Any],
    reference: Mapping[str, np.ndarray],
    initial_blocked_mask: np.ndarray,
    frame: Any,
    frame_count: int,
    out_root: Path,
    T_base_cam: np.ndarray,
) -> List[Dict[str, Any]]:
    rows = []
    for idx in range(max(int(frame_count), 0)):
        rows.append(
            _process_one_frame(
                bridge=bridge,
                classifier=classifier,
                handle=handle,
                accepted_reference=accepted_reference,
                reference=reference,
                initial_blocked_mask=initial_blocked_mask,
                frame=frame,
                frame_id=idx,
                out_root=out_root,
                T_base_cam=T_base_cam,
            )
        )
    return rows


def _run_live_frames(
    *,
    bridge: CameraFrameMapUpdateBridge,
    classifier: ReferenceBlockageClassifier,
    handle: Any,
    accepted_reference: Mapping[str, Any],
    reference: Mapping[str, np.ndarray],
    initial_blocked_mask: np.ndarray,
    frame_count: int,
    out_root: Path,
    T_base_cam: np.ndarray,
    camera_ns: str,
    camera_name: str,
    sync_slop: float,
    sync_queue: int,
    msg_conversion: str,
    live_timeout_sec: float,
) -> tuple[str, List[Dict[str, Any]]]:
    try:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from preprocessing import CameraPreprocessor
    except Exception as exc:
        _write_json(out_root / "live_camera_error.json", {"stage": "import", "error": str(exc)})
        return "HARDWARE_NOT_AVAILABLE", []

    preprocessor = None
    executor = None
    rows: List[Dict[str, Any]] = []
    try:
        rclpy.init(args=None)
        preprocessor = CameraPreprocessor(
            camera_namespace=camera_ns,
            camera_name=camera_name,
            sync_slop=float(sync_slop),
            sync_queue=int(sync_queue),
            msg_conversion=str(msg_conversion),
        )
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(preprocessor)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        if not preprocessor.wait_for_ready(timeout=float(live_timeout_sec)):
            _write_json(out_root / "live_camera_error.json", {"stage": "wait_for_ready_timeout"})
            return "HARDWARE_NOT_AVAILABLE", []
        for idx in range(max(int(frame_count), 0)):
            frame = preprocessor.wait_for_frame(timeout=0.5)
            if frame is None:
                continue
            rows.append(
                _process_one_frame(
                    bridge=bridge,
                    classifier=classifier,
                    handle=handle,
                    accepted_reference=accepted_reference,
                    reference=reference,
                    initial_blocked_mask=initial_blocked_mask,
                    frame=frame,
                    frame_id=idx,
                    out_root=out_root,
                    T_base_cam=T_base_cam,
                )
            )
        return "live_camera", rows
    finally:
        if executor is not None:
            executor.shutdown()
        if preprocessor is not None:
            preprocessor.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


def _process_one_frame(
    *,
    bridge: CameraFrameMapUpdateBridge,
    classifier: ReferenceBlockageClassifier,
    handle: Any,
    accepted_reference: Mapping[str, Any],
    reference: Mapping[str, np.ndarray],
    initial_blocked_mask: np.ndarray,
    frame: Any,
    frame_id: int,
    out_root: Path,
    T_base_cam: np.ndarray,
) -> Dict[str, Any]:
    adapter_frame_for_export = frame_to_adapter_frame(frame)
    raw_depth_m = np.asarray(adapter_frame_for_export["depth"], dtype=np.float64)
    unknown_candidate_mask = speed_prefilter_unknown_candidate_mask(
        raw_depth_m,
        max_depth_m=bridge.depth_relevant_max_m,
        crop_half_pixels=bridge.crop_half_pixels,
    )
    _, depth_m_for_export, _ = apply_speed_prefilter(
        None,
        raw_depth_m,
        max_depth_m=bridge.depth_relevant_max_m,
        crop_half_pixels=bridge.crop_half_pixels,
    )
    intrinsics_for_export = adapter_frame_for_export["intrinsics"]
    result = bridge.process(
        frame,
        [],
        handle,
        mu_min=DEFAULT_MU_MIN,
        T_base_cam=T_base_cam,
        sequence_id=int(frame_id),
    )
    report = classifier.classify(
        result.snapshot,
        s_table=reference["s"],
        xz_table=reference["xz"],
        current_s_m=0.0,
        previous_blocked_mask=initial_blocked_mask,
        accepted_reference=accepted_reference,
    )
    fov_mask, fov_stats = _compute_fov_mask(
        frame=frame,
        bridge=bridge,
        handle=handle,
        T_base_cam=T_base_cam,
    )
    goal_diagnostics = _goal_source_diagnostics(
        snapshot=result.snapshot,
        reference=reference,
        classifier=classifier,
        current_s_m=0.0,
        fov_mask=fov_mask,
    )
    report_dict = report.as_dict()
    frame_path = out_root / "frames" / f"frame_{int(frame_id):04d}.npz"
    frame_metadata_path = out_root / "frames" / f"frame_{int(frame_id):04d}.json"
    np.savez(
        frame_path,
        **_frame_npz_payload(
            result.snapshot,
            T_base_cam=T_base_cam,
            fov_mask=fov_mask,
            depth_m=depth_m_for_export,
            unknown_candidate_mask=unknown_candidate_mask,
        ),
    )
    frame_metadata = {
        "frame_id": int(frame_id),
        "runtime_mode": RUNTIME_MODE,
        "frame_artifact": str(frame_path.name),
        "frame_metadata_artifact": str(frame_metadata_path.name),
        "robot_command_endpoint_touched": False,
        "control_module_called_in_live_loop": False,
        "classifier_current_s_m": 0.0,
        "map_geometry": _map_geometry(result.snapshot.handle),
        "intrinsics": _intrinsics_metadata(intrinsics_for_export),
        "depth_geometry_config": _depth_geometry_config_metadata(bridge.depth_geometry.config),
        "snapshot_stats": dict(result.snapshot.stats),
        "result_stats": dict(result.stats),
        "request_stats": dict(result.request.adapter_stats),
        "validation": dict(result.validation),
        "classifier_report": report_dict,
        "goal_diagnostics": goal_diagnostics,
        "fov_stats": fov_stats,
        "T_base_cam": np.asarray(T_base_cam, dtype=np.float64),
    }
    _write_json(frame_metadata_path, frame_metadata)
    goal_counts = goal_diagnostics.get("goal_corridor_counts", {})
    goal_status_counts = goal_diagnostics.get("goal_status_neighborhood_counts", {})
    row = {
        "frame_id": int(frame_id),
        "event_class": str(report.event_class),
        "replanning_triggered": bool(report.replanning_triggered),
        "reason": str(report.reason),
        "reference_blocked_source": str(report_dict.get("reference_blocked_source", "none")),
        "reference_blocked_count": int(report.reference_blocked_count),
        "path_collision_is_collision": bool(report.path_collision_is_collision),
        "snapshot_valid": bool(result.validation.get("snapshot_valid", False)),
        "snapshot_blocked_cells": float(result.stats.get("snapshot_blocked_cells", 0.0)),
        "adapter_latency_ms": float(result.stats.get("adapter_latency_ms", 0.0)),
        "speed_prefilter_enabled": bool(result.stats.get("speed_prefilter_enabled", False)),
        "speed_prefilter_total_drop_ratio": float(result.stats.get("speed_prefilter_total_drop_ratio", 0.0)),
        "frame_artifact": str(frame_path.name),
        "frame_metadata_artifact": str(frame_metadata_path.name),
        "goal_cell_ix": int(goal_diagnostics.get("goal_cell", [-1, -1])[0]),
        "goal_cell_iz": int(goal_diagnostics.get("goal_cell", [-1, -1])[1]),
        "goal_corridor_blocked_count": int(goal_counts.get("blocked", 0)),
        "goal_corridor_cell_count": int(goal_counts.get("cell_count", 0)),
        "goal_corridor_occupied_count": int(goal_counts.get("occupied", 0)),
        "goal_corridor_target_count": int(goal_counts.get("target", 0)),
        "goal_corridor_unknown_count": int(goal_counts.get("unknown", 0)),
        "goal_corridor_occluded_count": int(goal_counts.get("occluded", 0)),
        "goal_corridor_inflated_count": int(goal_counts.get("inflated", 0)),
        "goal_corridor_sensor_inflated_count": int(goal_counts.get("sensor_inflated", 0)),
        "goal_corridor_occlusion_inflated_count": int(goal_counts.get("occlusion_inflated", 0)),
        "goal_corridor_base_feasible_count": int(goal_counts.get("base_feasible", 0)),
        "goal_corridor_base_feasible_exclusion_count": int(goal_counts.get("base_feasible_exclusion", 0)),
        "goal_corridor_final_active_count": int(goal_counts.get("final_active", 0)),
        "goal_corridor_fov_count": int(goal_counts.get("fov", 0)),
        "goal_corridor_fov_exclusion_count": int(goal_counts.get("fov_exclusion", 0)),
        "goal_corridor_residual_unattributed_count": int(goal_counts.get("residual_blocked_not_known_layer", 0)),
        "goal_status_neighborhood_blocked_count": int(goal_status_counts.get("blocked", 0)),
        "goal_status_neighborhood_final_active_count": int(goal_status_counts.get("final_active", 0)),
        "dominant_goal_corridor_source": str(goal_diagnostics.get("dominant_goal_corridor_source", "unavailable")),
        "fov_mask_available": bool(fov_stats.get("fov_mask_available", False)),
        "robot_command_endpoint_touched": False,
    }
    return row


def _frame_npz_payload(
    snapshot: Any,
    *,
    T_base_cam: np.ndarray,
    fov_mask: np.ndarray | None,
    depth_m: np.ndarray,
    unknown_candidate_mask: np.ndarray,
) -> Dict[str, np.ndarray]:
    shape = tuple(snapshot.handle.shape)
    payload: Dict[str, np.ndarray] = {
        "blocked_mask": np.asarray(snapshot.blocked_mask, dtype=bool),
        "final_active_mask": np.asarray(snapshot.final_active_mask, dtype=bool),
        "base_feasible_mask": np.asarray(snapshot.base_feasible_mask, dtype=bool),
        "occupied_mask": np.asarray(snapshot.occupied_mask, dtype=bool),
        "T_base_cam": np.asarray(T_base_cam, dtype=np.float64),
        "depth_m": np.asarray(depth_m, dtype=np.float32),
        "unknown_candidate_mask": np.asarray(unknown_candidate_mask, dtype=bool),
    }
    for name, mask in sorted(dict(snapshot.layer_masks).items()):
        payload[f"layer_{_safe_key(name)}"] = np.asarray(mask, dtype=bool)
    payload.setdefault("layer_occupied", np.asarray(snapshot.occupied_mask, dtype=bool))
    payload.setdefault("layer_blocked", np.asarray(snapshot.blocked_mask, dtype=bool))
    payload["fov_mask"] = (
        np.asarray(fov_mask, dtype=bool)
        if fov_mask is not None
        else np.zeros(shape, dtype=bool)
    )
    return payload


def _intrinsics_metadata(intrinsics: Any) -> Dict[str, Any]:
    return {
        "fx": float(intrinsics.fx),
        "fy": float(intrinsics.fy),
        "cx": float(intrinsics.cx),
        "cy": float(intrinsics.cy),
        "width": int(intrinsics.width),
        "height": int(intrinsics.height),
    }


def _depth_geometry_config_metadata(config: Any) -> Dict[str, Any]:
    fields = (
        "pixel_band_half_width_px",
        "depth_min_m",
        "depth_max_m",
        "y_plane",
        "delta_y_static_m",
        "k_sigma",
        "min_unknown_component_pixels",
    )
    return {name: float(getattr(config, name)) for name in fields}


def _compute_fov_mask(
    *,
    frame: Any,
    bridge: CameraFrameMapUpdateBridge,
    handle: Any,
    T_base_cam: np.ndarray,
) -> tuple[np.ndarray | None, Dict[str, Any]]:
    try:
        adapter_frame = frame_to_adapter_frame(frame)
        cfg = bridge.depth_geometry.config
        mask = compute_task_plane_fov_mask(
            handle,
            adapter_frame["intrinsics"],  # type: ignore[arg-type]
            T_base_cam,
            y_plane=float(cfg.y_plane),
            d_min=float(cfg.depth_min_m),
            d_max_task=float(cfg.depth_max_m),
            pixel_band_half_width=float(cfg.pixel_band_half_width_px),
        )
        return np.asarray(mask, dtype=bool), {
            "fov_mask_available": True,
            "fov_observed_cells": int(np.count_nonzero(mask)),
            "fov_coverage_ratio": float(np.count_nonzero(mask) / max(mask.size, 1)),
            "fov_depth_min_m": float(cfg.depth_min_m),
            "fov_depth_max_m": float(cfg.depth_max_m),
            "fov_pixel_band_half_width_px": float(cfg.pixel_band_half_width_px),
        }
    except Exception as exc:
        return None, {
            "fov_mask_available": False,
            "fov_mask_error": str(exc),
        }


def _goal_source_diagnostics(
    *,
    snapshot: Any,
    reference: Mapping[str, np.ndarray],
    classifier: ReferenceBlockageClassifier,
    current_s_m: float,
    fov_mask: np.ndarray | None,
) -> Dict[str, Any]:
    s_arr = np.asarray(reference["s"], dtype=np.float64).reshape(-1)
    xz_arr = np.asarray(reference["xz"], dtype=np.float64).reshape(-1, 2)
    if s_arr.size == 0 or xz_arr.shape[0] != s_arr.size:
        return {"diagnostics_available": False, "reason": "invalid_reference"}

    handle = snapshot.handle
    length = float(s_arr[-1])
    current_s = float(np.clip(current_s_m, 0.0, max(length, 0.0)))
    goal_start = max(0.0, length - max(float(classifier.goal_window_m), 0.0))
    goal_pts = _sample_range_local(s_arr, xz_arr, goal_start, length, include_nearest=True)
    current_pts = _sample_range_local(s_arr, xz_arr, current_s, current_s, include_nearest=True)
    future_pts = _sample_range_local(s_arr, xz_arr, current_s, length, include_nearest=True)
    goal_xz = np.asarray(xz_arr[-1], dtype=np.float64)
    goal_cell = _cell_from_xz_local(handle, goal_xz)
    current_cell = _cell_from_xz_local(handle, current_pts[0]) if current_pts.size else None
    local_radius = max(2.0 * float(handle.resolution_m), float(classifier.corridor_radius_m))

    goal_corridor_cells = _cells_from_points(handle, goal_pts, float(classifier.corridor_radius_m))
    future_corridor_cells = _cells_from_points(handle, future_pts, float(classifier.corridor_radius_m))
    goal_status_cells = (
        _neighborhood_cells_local(handle, goal_cell, local_radius)
        if goal_cell is not None
        else []
    )
    current_pose_cells = (
        _neighborhood_cells_local(handle, current_cell, float(classifier.current_pose_radius_m))
        if current_cell is not None
        else []
    )
    masks = _diagnostic_masks(snapshot, fov_mask=fov_mask)
    goal_counts = _count_sources(masks, goal_corridor_cells)
    status_counts = _count_sources(masks, goal_status_cells)
    current_counts = _count_sources(masks, current_pose_cells)
    future_counts = _count_sources(masks, future_corridor_cells)
    return {
        "diagnostics_available": True,
        "current_s_m": current_s,
        "goal_xz": goal_xz,
        "goal_cell": [-1, -1] if goal_cell is None else [int(goal_cell[0]), int(goal_cell[1])],
        "current_cell": [-1, -1] if current_cell is None else [int(current_cell[0]), int(current_cell[1])],
        "goal_window_m": float(classifier.goal_window_m),
        "corridor_radius_m": float(classifier.corridor_radius_m),
        "goal_status_neighborhood_radius_m": float(local_radius),
        "goal_corridor_cells": [[int(ix), int(iz)] for ix, iz in sorted(goal_corridor_cells)],
        "goal_status_neighborhood_cells": [[int(ix), int(iz)] for ix, iz in sorted(goal_status_cells)],
        "current_pose_cells": [[int(ix), int(iz)] for ix, iz in sorted(current_pose_cells)],
        "goal_corridor_counts": goal_counts,
        "goal_status_neighborhood_counts": status_counts,
        "current_pose_counts": current_counts,
        "future_reference_corridor_counts": future_counts,
        "dominant_goal_corridor_source": _dominant_source(goal_counts),
    }


def _diagnostic_masks(snapshot: Any, *, fov_mask: np.ndarray | None) -> Dict[str, np.ndarray]:
    masks: Dict[str, np.ndarray] = {
        "blocked": np.asarray(snapshot.blocked_mask, dtype=bool),
        "final_active": np.asarray(snapshot.final_active_mask, dtype=bool),
        "base_feasible": np.asarray(snapshot.base_feasible_mask, dtype=bool),
        "occupied": np.asarray(snapshot.occupied_mask, dtype=bool),
    }
    for name, mask in dict(snapshot.layer_masks).items():
        masks[_safe_key(name)] = np.asarray(mask, dtype=bool)
    if fov_mask is not None:
        masks["fov"] = np.asarray(fov_mask, dtype=bool)
    return masks


def _count_sources(masks: Mapping[str, np.ndarray], cells: Sequence[tuple[int, int]]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for name, mask in masks.items():
        counts[str(name)] = int(sum(1 for ix, iz in cells if bool(mask[ix, iz])))
    blocked = counts.get("blocked", 0)
    known = np.zeros(next(iter(masks.values())).shape, dtype=bool)
    for name in (
        "occupied",
        "target",
        "unknown",
        "occluded",
        "inflated",
        "sensor_inflated",
        "occlusion_inflated",
    ):
        if name in masks:
            known |= np.asarray(masks[name], dtype=bool)
    counts["residual_blocked_not_known_layer"] = int(
        sum(1 for ix, iz in cells if bool(masks["blocked"][ix, iz]) and not bool(known[ix, iz]))
    )
    counts["base_feasible_exclusion"] = int(
        sum(
            1
            for ix, iz in cells
            if "base_feasible" in masks and not bool(masks["base_feasible"][ix, iz])
        )
    )
    counts["fov_exclusion"] = int(
        sum(1 for ix, iz in cells if "fov" in masks and not bool(masks["fov"][ix, iz]))
    )
    counts["cell_count"] = int(len(cells))
    counts["blocked_without_explanation"] = max(
        0,
        int(blocked)
        - max(
            counts.get("occupied", 0),
            counts.get("target", 0),
            counts.get("unknown", 0),
            counts.get("occluded", 0),
            counts.get("inflated", 0),
            counts.get("sensor_inflated", 0),
            counts.get("occlusion_inflated", 0),
        ),
    )
    return counts


def _dominant_source(counts: Mapping[str, int]) -> str:
    candidates = [
        "occupied",
        "target",
        "unknown",
        "occluded",
        "sensor_inflated",
        "occlusion_inflated",
        "inflated",
        "base_feasible_exclusion",
        "fov_exclusion",
        "residual_blocked_not_known_layer",
    ]
    best = max(candidates, key=lambda name: int(counts.get(name, 0)))
    return best if int(counts.get(best, 0)) > 0 else "none"


def _sample_range_local(
    s_arr: np.ndarray,
    xz_arr: np.ndarray,
    s0: float,
    s1: float,
    *,
    include_nearest: bool,
) -> np.ndarray:
    lo = min(float(s0), float(s1))
    hi = max(float(s0), float(s1))
    mask = (s_arr >= lo - 1.0e-12) & (s_arr <= hi + 1.0e-12)
    pts = xz_arr[mask]
    if pts.size == 0 and include_nearest:
        idx = int(np.argmin(np.abs(s_arr - 0.5 * (lo + hi))))
        pts = xz_arr[idx:idx + 1]
    return np.asarray(pts, dtype=np.float64).reshape(-1, 2)


def _cells_from_points(handle: Any, xz: np.ndarray, radius_m: float) -> set[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    for point in _expand_points_local(xz, radius_m):
        cell = _cell_from_xz_local(handle, point)
        if cell is not None:
            cells.add(cell)
    return cells


def _expand_points_local(xz: np.ndarray, radius_m: float) -> np.ndarray:
    pts = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
    r = max(float(radius_m), 0.0)
    if r <= 1.0e-12:
        return pts
    offsets = np.array(
        [
            [0.0, 0.0],
            [r, 0.0],
            [-r, 0.0],
            [0.0, r],
            [0.0, -r],
            [0.70710678 * r, 0.70710678 * r],
            [0.70710678 * r, -0.70710678 * r],
            [-0.70710678 * r, 0.70710678 * r],
            [-0.70710678 * r, -0.70710678 * r],
        ],
        dtype=np.float64,
    )
    return (pts[:, None, :] + offsets[None, :, :]).reshape(-1, 2)


def _cell_from_xz_local(handle: Any, xz: np.ndarray) -> tuple[int, int] | None:
    ix = int(round((float(xz[0]) - float(handle.x0)) / float(handle.resolution_m)))
    iz = int(round((float(xz[1]) - float(handle.z0)) / float(handle.resolution_m)))
    n_x, n_z = handle.shape
    if ix < 0 or ix >= n_x or iz < 0 or iz >= n_z:
        return None
    return ix, iz


def _neighborhood_cells_local(handle: Any, center: tuple[int, int], radius_m: float) -> set[tuple[int, int]]:
    n_x, n_z = handle.shape
    radius_cells = int(np.ceil(max(float(radius_m), 0.0) / max(float(handle.resolution_m), 1.0e-12)))
    cx, cz = center
    cells: set[tuple[int, int]] = set()
    for ix in range(max(0, cx - radius_cells), min(n_x, cx + radius_cells + 1)):
        for iz in range(max(0, cz - radius_cells), min(n_z, cz + radius_cells + 1)):
            dx = (ix - cx) * float(handle.resolution_m)
            dz = (iz - cz) * float(handle.resolution_m)
            if dx * dx + dz * dz <= radius_m * radius_m + 1.0e-12:
                cells.add((ix, iz))
    return cells


def _map_geometry(handle: Any) -> Dict[str, Any]:
    return {
        "shape": [int(v) for v in handle.shape],
        "resolution_m": float(handle.resolution_m),
        "x0": float(handle.x0),
        "z0": float(handle.z0),
        "target_y": float(handle.target_y),
        "tag": str(handle.tag),
        "map_path": str(getattr(handle, "map_path", "")),
        "meta_path": str(getattr(handle, "meta_path", "")),
    }


def _safe_key(name: object) -> str:
    return "".join(ch if ch.isalnum() else "_" for ch in str(name).strip().lower()).strip("_")


def make_synthetic_frame() -> SimpleNamespace:
    depth = np.full((64, 64), 2.0, dtype=np.float32)
    intrinsics = np.array(
        [[80.0, 0.0, 32.0], [0.0, 80.0, 32.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return SimpleNamespace(depth=depth, intrinsics=intrinsics, timestamp=time.time(), received_timestamp=time.time())


def save_synthetic_frame(path: Path) -> None:
    frame = make_synthetic_frame()
    np.savez(path, depth=frame.depth, intrinsics=frame.intrinsics, timestamp=float(frame.timestamp))


def _load_frame_npz(path: Path) -> SimpleNamespace:
    if not path.exists():
        raise FileNotFoundError(str(path))
    data = np.load(path, allow_pickle=False)
    timestamp = float(data["timestamp"]) if "timestamp" in data.files else time.time()
    return SimpleNamespace(
        depth=np.asarray(data["depth"]),
        intrinsics=np.asarray(data["intrinsics"]),
        timestamp=timestamp,
        received_timestamp=timestamp,
    )


def _load_q_traj(path: Path) -> np.ndarray:
    if not path.exists():
        return np.zeros((0, 3), dtype=np.float64)
    rows: List[List[float]] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        names = reader.fieldnames or []
        q_names = [name for name in names if name in {"q2", "q3", "q5"}]
        if len(q_names) < 3:
            q_names = names[-3:]
        for row in reader:
            try:
                rows.append([float(row[name]) for name in q_names[:3]])
            except Exception:
                continue
    return np.asarray(rows, dtype=np.float64).reshape(-1, 3) if rows else np.zeros((0, 3), dtype=np.float64)


def _event_counts(rows: Iterable[Mapping[str, Any]]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for row in rows:
        key = str(row.get("event_class", "UNKNOWN"))
        counts[key] = counts.get(key, 0) + 1
    return counts


def _source_counts(rows: Iterable[Mapping[str, Any]]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for row in rows:
        key = str(row.get("reference_blocked_source", "none"))
        counts[key] = counts.get(key, 0) + 1
    return counts


def _nested_get(data: Mapping[str, Any], path: tuple[str, ...], default: Any) -> Any:
    cur: Any = data
    for key in path:
        if not isinstance(cur, Mapping):
            return default
        cur = cur.get(key)
    return default if cur is None else cur


def _write_json(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(_jsonable(data), f, indent=2, ensure_ascii=False)


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    rows = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(dict(row))


def _jsonable(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--out", required=True)
    parser.add_argument("--use-saved-frame", default=None)
    parser.add_argument("--write-synthetic-frame", default=None)
    parser.add_argument("--map-path", default=None)
    parser.add_argument("--camera-ns", default="gripper_camera")
    parser.add_argument("--camera-name", default="gripper_camera")
    parser.add_argument("--sync-slop", type=float, default=0.03)
    parser.add_argument("--sync-queue", type=int, default=10)
    parser.add_argument("--msg-conversion", default="auto")
    parser.add_argument("--live-timeout-sec", type=float, default=12.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.write_synthetic_frame:
        save_synthetic_frame(Path(args.write_synthetic_frame))
    summary = run_passive_decision(
        out=args.out,
        frames=int(args.frames),
        use_saved_frame=args.use_saved_frame,
        map_path=args.map_path,
        camera_ns=args.camera_ns,
        camera_name=args.camera_name,
        sync_slop=float(args.sync_slop),
        sync_queue=int(args.sync_queue),
        msg_conversion=str(args.msg_conversion),
        live_timeout_sec=float(args.live_timeout_sec),
    )
    print(json.dumps(_jsonable(summary), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
