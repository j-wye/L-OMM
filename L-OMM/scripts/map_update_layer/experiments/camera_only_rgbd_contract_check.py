#!/usr/bin/env python3
"""Validate the CameraPreprocessor -> map-update bridge contract.

Default execution is synthetic/stored-input only.  Passing ``--live-samples``
uses the existing ``CameraPreprocessor`` class as the only RGB-D ingress and
does not command the robot, arm, gripper, or mobile base.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import threading
import time
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, "..", ".."))

if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(0, CONTROL_MODULE_DIR)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(1, SCRIPTS_DIR)

from constants import DEFAULT_MU_MIN, Y_PLANE_FIXED  # noqa: E402
from map_handle import MapHandle  # noqa: E402
from map_update_layer import CameraFrameMapUpdateBridge, Detection2D  # noqa: E402
from map_update_layer.perception_to_map import nominal_t_base_cam  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Check CameraPreprocessor FrameData reuse for map update. "
            "Synthetic by default; --live-samples uses CameraPreprocessor only."
        )
    )
    parser.add_argument("--out", default=None)
    parser.add_argument("--iterations", type=int, default=30)
    parser.add_argument("--live-samples", type=int, default=0)
    parser.add_argument("--live-timeout-sec", type=float, default=12.0)
    parser.add_argument("--camera-ns", default="gripper_camera")
    parser.add_argument("--camera-name", default="gripper_camera")
    parser.add_argument("--sync-slop", type=float, default=0.01)
    parser.add_argument("--sync-queue", type=int, default=5)
    parser.add_argument("--msg-conversion", default="direct", choices=("direct", "cv_bridge"))
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "camera_only_rgbd_contract_check"))
    os.makedirs(out_root, exist_ok=True)

    handle = _synthetic_handle()
    bridge = CameraFrameMapUpdateBridge()
    frame = _synthetic_frame()
    detections = _synthetic_evidence()
    T = nominal_t_base_cam(Y_PLANE_FIXED)
    T[:3, 3] = np.array([0.0, Y_PLANE_FIXED, 0.50], dtype=np.float64)

    result = bridge.process(
        frame,
        detections,
        handle,
        mu_min=DEFAULT_MU_MIN,
        T_base_cam=T,
        sequence_id=1,
        map_update_hz=10.0,
    )
    single_checks = _single_semantic_checks(bridge, frame, handle, T)
    latency_rows = _adapter_latency_rows(bridge, frame, detections, T, int(args.iterations))
    frame_rows = [_frame_stats_row("synthetic", frame, result.stats)]

    live_status = "HARDWARE_NOT_REQUESTED"
    live_rows: List[Dict[str, Any]] = []
    if int(args.live_samples) > 0:
        live_status, live_rows = _try_live_camera_samples(args, out_root, handle)
        frame_rows.extend(live_rows)

    adapter_latencies = np.asarray([float(row["adapter_latency_ms"]) for row in latency_rows], dtype=np.float64)
    adapter_median = float(np.median(adapter_latencies)) if adapter_latencies.size else float("inf")
    adapter_p99 = float(np.percentile(adapter_latencies, 99.0)) if adapter_latencies.size else float("inf")
    latency_artifact_status = _latest_latency_artifact_status()
    local_latency_status = "CONTRACTUAL_PASS" if adapter_median <= 10.0 and adapter_p99 <= 33.0 else "FAIL"
    latency_status = latency_artifact_status if latency_artifact_status != "UNKNOWN" else local_latency_status
    yband_status = _latest_pass_status("yband_param_sweep_summary.json")
    replan_status = _latest_pass_status("replan_trigger_metrics_summary.json")

    snapshot_stats = _jsonable(dict(result.snapshot.stats))
    adapter_stats = dict(result.request.adapter_stats)
    _write_csv(os.path.join(out_root, "frame_stats.csv"), frame_rows)
    _write_csv(os.path.join(out_root, "adapter_latency.csv"), latency_rows)
    _write_json(os.path.join(out_root, "snapshot_stats.json"), snapshot_stats)

    summary: Dict[str, Any] = {
        "mode": "camera_preprocessor_map_update_bridge_contract",
        "explicitly_no_new_camera_subscriber": True,
        "explicitly_no_robot_command": True,
        "camera_preprocessor_reused": bool(result.stats.get("camera_preprocessor_reused", False)),
        "new_camera_subscriber_added": bool(result.stats.get("new_camera_subscriber_added", True)),
        "frame_timestamp_shared": bool(result.stats.get("frame_timestamp_shared", False)),
        "depth_unit": str(result.stats.get("depth_unit", "")),
        "invalid_depth_ratio": float(result.stats.get("invalid_depth_ratio", 1.0)),
        "detections_in": int(result.stats.get("detections_in", 0)),
        "detections_blocked": int(result.stats.get("detections_blocked", 0)),
        "target_blocked": bool(single_checks["target_blocked"] and result.stats.get("target_blocked", False)),
        "support_surface_blocked": bool(single_checks["support_surface_blocked"] and result.stats.get("support_surface_blocked", False)),
        "nearby_geometry_blocked": bool(single_checks["nearby_geometry_blocked"] and result.stats.get("nearby_geometry_blocked", False)),
        "filtered_out_of_task_count": int(result.stats.get("filtered_out_of_task_count", 0)),
        "adapter_latency_ms_median": adapter_median,
        "adapter_latency_ms_p99": adapter_p99,
        "snapshot_valid": float(result.validation.get("snapshot_valid", 0.0)),
        "latency_status": latency_status,
        "latency_artifact_status": latency_artifact_status,
        "local_bridge_latency_status": local_latency_status,
        "yband_regression": yband_status,
        "replan_trigger_regression": replan_status,
        "camera_only_validation": live_status,
        "live_sample_count": len(live_rows),
        "live_transform_source": (
            "nominal_t_base_cam_diagnostic_only"
            if int(args.live_samples) > 0 and live_rows
            else "not_applicable"
        ),
        "live_calibrated_tf_contract": False if int(args.live_samples) > 0 and live_rows else None,
        "depth_geometry_enabled": bool(adapter_stats.get("depth_geometry_enabled", False)),
        "depth_geometry_occupied_rect_count": int(adapter_stats.get("depth_geometry_occupied_rect_count", 0)),
        "depth_geometry_unknown_rect_count": int(adapter_stats.get("depth_geometry_unknown_rect_count", 0)),
        "depth_geometry_added_occupied_cells": int(adapter_stats.get("depth_geometry_added_occupied_cells", 0)),
        "depth_geometry_added_unknown_cells": int(adapter_stats.get("depth_geometry_added_unknown_cells", 0)),
        "depth_geometry_latency_ms": float(adapter_stats.get("depth_geometry_latency_ms", 0.0)),
        "required_claim": (
            "Map-update latency is real-time under task-relevant active evidence, "
            "not under arbitrary full-frame 50-object processing."
        ),
        "artifact_paths": {
            "summary": os.path.join(out_root, "summary.json"),
            "frame_stats": os.path.join(out_root, "frame_stats.csv"),
            "adapter_latency": os.path.join(out_root, "adapter_latency.csv"),
            "snapshot_stats": os.path.join(out_root, "snapshot_stats.json"),
        },
    }
    _write_json(os.path.join(out_root, "summary.json"), _jsonable(summary))
    print(
        "[camera_only_rgbd_contract_check] "
        f"snapshot_valid={summary['snapshot_valid']} "
        f"latency={summary['latency_status']} "
        f"camera={summary['camera_only_validation']} "
        f"out={out_root}"
    )
    required_pass = (
        bool(summary["camera_preprocessor_reused"])
        and not bool(summary["new_camera_subscriber_added"])
        and bool(summary["frame_timestamp_shared"])
        and bool(summary["target_blocked"])
        and bool(summary["support_surface_blocked"])
        and bool(summary["nearby_geometry_blocked"])
        and float(summary["snapshot_valid"]) == 1.0
        and str(summary["local_bridge_latency_status"]) == "CONTRACTUAL_PASS"
    )
    return 0 if required_pass else 1


def _synthetic_handle() -> MapHandle:
    shape = (110, 140)
    mu_grid = np.full(shape, 0.05, dtype=np.float64)
    pitch_grid = np.zeros(shape, dtype=np.float64)
    return MapHandle(
        map_path="synthetic",
        meta_path="synthetic",
        mu_grid=mu_grid,
        pitch_grid=pitch_grid,
        q_grid=None,
        meta={"mu_min_runtime": DEFAULT_MU_MIN},
        resolution_m=0.01,
        x0=0.0,
        z0=-0.45,
        target_y=Y_PLANE_FIXED,
        tag="camera_bridge_synthetic",
    )


def _synthetic_frame() -> SimpleNamespace:
    height, width = 480, 640
    depth = np.zeros((height, width), dtype=np.uint16)
    for bbox, depth_mm in [
        ((300, 212, 340, 248), 350),
        ((292, 255, 348, 286), 360),
        ((352, 218, 392, 252), 370),
    ]:
        u0, v0, u1, v1 = bbox
        depth[v0:v1 + 1, u0:u1 + 1] = depth_mm
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    K = np.array(
        [
            [615.0, 0.0, 319.5],
            [0.0, 615.0, 239.5],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return SimpleNamespace(
        rgb=rgb,
        depth=depth,
        intrinsics=K,
        distortion=np.zeros(5, dtype=np.float64),
        distortion_model="plumb_bob",
        is_bgr=False,
        timestamp=123.456,
        received_timestamp=123.466,
    )


def _synthetic_evidence() -> List[Detection2D]:
    return [
        Detection2D("target", (300, 212, 340, 248), source_id="target", score=0.95),
        Detection2D("occupied", (292, 255, 348, 286), source_id="supporting_surface", score=0.90),
        Detection2D("occupied", (352, 218, 392, 252), source_id="nearby_geometry", score=0.82),
    ]


def _single_semantic_checks(
    bridge: CameraFrameMapUpdateBridge,
    frame: SimpleNamespace,
    handle: MapHandle,
    T: np.ndarray,
) -> Dict[str, bool]:
    checks: Dict[str, bool] = {}
    cases = {
        "target_blocked": [Detection2D("target", (300, 212, 340, 248), source_id="target")],
        "support_surface_blocked": [Detection2D("occupied", (292, 255, 348, 286), source_id="supporting_surface")],
        "nearby_geometry_blocked": [Detection2D("occupied", (352, 218, 392, 252), source_id="nearby_geometry")],
    }
    for name, dets in cases.items():
        result = bridge.process(frame, dets, handle, mu_min=DEFAULT_MU_MIN, T_base_cam=T)
        if name == "target_blocked":
            checks[name] = float(result.snapshot.stats.get("target_cells", 0.0)) > 0.0
        else:
            checks[name] = float(result.snapshot.stats.get("occupied_cells", 0.0)) > 0.0
    return checks


def _adapter_latency_rows(
    bridge: CameraFrameMapUpdateBridge,
    frame: SimpleNamespace,
    detections: Sequence[Detection2D],
    T: np.ndarray,
    iterations: int,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for idx in range(max(int(iterations), 1)):
        request = bridge.build_request(frame, detections, T_base_cam=T, sequence_id=idx)
        stats = dict(request.adapter_stats)
        rows.append(
            {
                "iteration": idx,
                "detections_in": int(stats.get("detections_in", 0)),
                "detections_blocked": int(stats.get("detections_blocked", 0)),
                "image_detection_count_active": float(stats.get("image_detection_count_active", 0.0)),
                "adapter_latency_ms": float(stats.get("adapter_latency_ms", 0.0)),
            }
        )
    return rows


def _frame_stats_row(mode: str, frame: Any, stats: Mapping[str, Any]) -> Dict[str, Any]:
    depth = np.asarray(frame.depth)
    timestamp = float(getattr(frame, "timestamp", 0.0))
    received = float(getattr(frame, "received_timestamp", timestamp))
    return {
        "mode": mode,
        "timestamp_s": timestamp,
        "received_timestamp_s": received,
        "buffer_age_ms": max((received - timestamp) * 1000.0, 0.0),
        "height": int(depth.shape[0]),
        "width": int(depth.shape[1]),
        "depth_dtype": str(depth.dtype),
        "depth_unit": str(stats.get("depth_unit", "")),
        "invalid_depth_ratio": float(stats.get("invalid_depth_ratio", 1.0)),
    }


def _try_live_camera_samples(args: argparse.Namespace, out_root: str, handle: MapHandle) -> Tuple[str, List[Dict[str, Any]]]:
    try:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from preprocessing import CameraPreprocessor
    except Exception as exc:
        _write_json(os.path.join(out_root, "live_camera_error.json"), {"error": str(exc), "stage": "import"})
        return "HARDWARE_NOT_AVAILABLE", []

    rows: List[Dict[str, Any]] = []
    preprocessor = None
    executor = None
    try:
        rclpy.init(args=None)
        preprocessor = CameraPreprocessor(
            camera_namespace=args.camera_ns,
            camera_name=args.camera_name,
            sync_slop=float(args.sync_slop),
            sync_queue=int(args.sync_queue),
            msg_conversion=str(args.msg_conversion),
        )
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(preprocessor)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        if not preprocessor.wait_for_ready(timeout=float(args.live_timeout_sec)):
            _write_json(
                os.path.join(out_root, "live_camera_error.json"),
                {
                    "error": "CameraPreprocessor did not receive intrinsics and synchronized RGB-D frame before timeout",
                    "stage": "wait_for_ready_timeout",
                    "camera_namespace": str(args.camera_ns),
                    "camera_name": str(args.camera_name),
                    "timeout_sec": float(args.live_timeout_sec),
                },
            )
            return "HARDWARE_NOT_AVAILABLE", []
        bridge = CameraFrameMapUpdateBridge()
        T = nominal_t_base_cam(Y_PLANE_FIXED)
        T[:3, 3] = np.array([0.0, Y_PLANE_FIXED, 0.50], dtype=np.float64)
        for idx in range(int(args.live_samples)):
            frame = preprocessor.wait_for_frame(timeout=0.5)
            if frame is None:
                continue
            det = _center_detection_from_depth(frame)
            result = bridge.process(frame, [det], handle, mu_min=DEFAULT_MU_MIN, T_base_cam=T, sequence_id=idx)
            row = _frame_stats_row("live_camera", frame, result.stats)
            row.update(
                {
                    "sync_dt_mean_ms": float(preprocessor.sync_dt_mean_ms),
                    "sync_dt_max_ms": float(preprocessor.sync_dt_max_ms),
                    "rgb_depth_frame_count": int(preprocessor.frame_count),
                    "snapshot_valid": float(result.validation.get("snapshot_valid", 0.0)),
                    "adapter_latency_ms": float(result.stats.get("adapter_latency_ms", 0.0)),
                    "transform_source": "nominal_t_base_cam_diagnostic_only",
                    "calibrated_tf_contract": False,
                }
            )
            rows.append(row)
        return ("PASS" if rows else "HARDWARE_NOT_AVAILABLE"), rows
    except Exception as exc:
        _write_json(os.path.join(out_root, "live_camera_error.json"), {"error": str(exc), "stage": "runtime"})
        return "HARDWARE_NOT_AVAILABLE", rows
    finally:
        try:
            if preprocessor is not None:
                preprocessor.destroy_node()
            if executor is not None:
                executor.shutdown(timeout_sec=1.0)
            if "rclpy" in locals() and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def _center_detection_from_depth(frame: Any) -> Detection2D:
    depth = np.asarray(frame.depth)
    h, w = depth.shape
    half_w = max(min(w, h) // 18, 8)
    u0 = max(w // 2 - half_w, 0)
    u1 = min(w // 2 + half_w, w - 1)
    v0 = max(h // 2 - half_w, 0)
    v1 = min(h // 2 + half_w, h - 1)
    return Detection2D("occupied", (u0, v0, u1, v1), source_id="camera_center_evidence")


def _latest_latency_artifact_status() -> str:
    latest = _latest_json_named("latency_sweep_summary.json")
    if latest is None:
        return "UNKNOWN"
    data = latest[1]
    if "p2_5_pass_mode" in data:
        return str(data["p2_5_pass_mode"])
    if bool(data.get("contractual_pass", False)):
        return "CONTRACTUAL_PASS"
    if bool(data.get("strict_pass", False)) or bool(data.get("pass", False)):
        return "PASS"
    return "FAIL"


def _latest_pass_status(filename: str) -> str:
    latest = _latest_json_named(filename)
    if latest is None:
        return "UNKNOWN"
    return "PASS" if bool(latest[1].get("pass", False)) else "FAIL"


def _latest_json_named(filename: str) -> Tuple[str, Dict[str, Any]] | None:
    path_root = os.path.join(PROJECT_ROOT, "path")
    if not os.path.isdir(path_root):
        return None
    matches: List[str] = []
    for root, _, files in os.walk(path_root):
        if filename in files:
            matches.append(os.path.join(root, filename))
    if not matches:
        return None
    newest = max(matches, key=lambda p: os.path.getmtime(p))
    with open(newest, "r", encoding="utf-8") as f:
        return newest, json.load(f)


def _write_csv(path: str, rows: List[Dict[str, Any]]) -> None:
    keys: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in keys:
                keys.append(key)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def _write_json(path: str, data: Any) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(_jsonable(data), f, indent=2, ensure_ascii=False)


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    return value


if __name__ == "__main__":
    raise SystemExit(main())
