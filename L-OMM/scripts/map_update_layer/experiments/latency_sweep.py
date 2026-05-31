#!/usr/bin/env python3
"""Synthetic Stage A-F latency sweep for PerceptionToMapAdapter."""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, "..", ".."))

if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(0, CONTROL_MODULE_DIR)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(1, SCRIPTS_DIR)

from constants import Y_PLANE_FIXED  # noqa: E402
from map_update_layer.camera_frame_bridge import apply_speed_prefilter  # noqa: E402
from map_update_layer.path_collision_monitor import PathCollisionMonitor  # noqa: E402
from map_update_layer.perception_to_map import (  # noqa: E402
    CameraIntrinsics,
    Detection2D,
    PerceptionToMapAdapter,
    nominal_t_base_cam,
)
from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams  # noqa: E402


LATENCY_KEYS = [
    "stage_a_image_prefilter_latency_ms",
    "stage_b_depth_pixels_latency_ms",
    "stage_c_backproject_transform_latency_ms",
    "stage_d_yband_filter_latency_ms",
    "stage_e_rect_semantic_latency_ms",
    "stage_f_request_assembly_latency_ms",
    "adapter_latency_ms",
]

COUNT_KEYS = [
    "image_detection_count_in",
    "image_detection_count_kept",
    "image_detection_count_prefilter_kept",
    "image_detection_count_active",
    "image_detection_count_pruned_by_topk",
    "max_active_detections_used",
]

EXTENSION_KEYS = [
    "prefilter_applied",
    "prefilter_drop_ratio",
    "sticky_update_latency_ms",
    "path_collision_check_latency_ms",
]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Measure synthetic PerceptionToMapAdapter latency. "
            "No live camera, ROS2 topic, or physical robot execution."
        )
    )
    parser.add_argument("--out", default=None)
    parser.add_argument("--iterations", type=int, default=10)
    parser.add_argument("--detection-counts", default="1,5,10,50")
    parser.add_argument("--mask-sizes", default="100x100,640x480,1280x720")
    parser.add_argument("--max-points-per-detection", type=int, default=8000)
    parser.add_argument(
        "--max-active-detections",
        type=int,
        default=10,
        help="Task-relevant active detection cap. Use 0 to disable pruning.",
    )
    parser.add_argument(
        "--enable-prefilter",
        action="store_true",
        help="Apply CameraFrameMapUpdateBridge speed prefilter before adapter timing.",
    )
    parser.add_argument("--depth-relevant-max-m", type=float, default=1.0)
    parser.add_argument("--crop-half-pixels", type=int, default=200)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "latency_sweep"))
    os.makedirs(out_root, exist_ok=True)
    counts = [int(v.strip()) for v in str(args.detection_counts).split(",") if v.strip()]
    sizes = [_parse_size(v) for v in str(args.mask_sizes).split(",") if v.strip()]
    rows: List[Dict[str, Any]] = []
    for width, height in sizes:
        for n_det in counts:
            raw: List[Dict[str, float]] = []
            for iteration in range(int(args.iterations)):
                stats = _run_one(
                    width,
                    height,
                    n_det,
                    iteration,
                    int(args.max_points_per_detection),
                    int(args.max_active_detections),
                    bool(args.enable_prefilter),
                    float(args.depth_relevant_max_m),
                    int(args.crop_half_pixels),
                )
                raw.append(stats)
                rows.append(
                    {
                        "width": width,
                        "height": height,
                        "detection_count": n_det,
                        "iteration": iteration,
                        **stats,
                    }
                )
    aggregate = _aggregate(rows)
    strict_pass = _strict_latency_pass(aggregate)
    contractual_pass = _contractual_latency_pass(aggregate, int(args.max_active_detections))
    summary = {
        "mode": "synthetic_adapter_latency_sweep",
        "explicitly_no_live_camera": True,
        "explicitly_no_ros2_topic": True,
        "explicitly_no_physical_robot_execution": True,
        "iterations": int(args.iterations),
        "detection_counts": counts,
        "mask_sizes": [{"width": w, "height": h} for w, h in sizes],
        "max_active_detections": int(args.max_active_detections),
        "prefilter_extension_enabled": bool(args.enable_prefilter),
        "depth_relevant_max_m": float(args.depth_relevant_max_m),
        "crop_half_pixels": int(args.crop_half_pixels),
        "aggregate": aggregate,
        "strict_pass": bool(strict_pass),
        "contractual_pass": bool(contractual_pass),
        "p2_5_pass_mode": "STRICT_PASS" if strict_pass else ("CONTRACTUAL_PASS" if contractual_pass else "FAIL"),
        "pass": bool(strict_pass or contractual_pass),
    }
    _write_csv(os.path.join(out_root, "latency_sweep_raw.csv"), rows)
    _write_csv(os.path.join(out_root, "latency_sweep_summary.csv"), aggregate)
    _write_json(os.path.join(out_root, "latency_sweep_summary.json"), summary)
    _write_latency_plot(os.path.join(out_root, "latency_sweep_end_to_end.png"), aggregate)
    print(
        "[latency_sweep] "
        f"rows={len(rows)} pass={summary['pass']} out={out_root}"
    )
    return 0 if summary["pass"] else 1


def _run_one(
    width: int,
    height: int,
    n_det: int,
    iteration: int,
    max_points: int,
    max_active_detections: int,
    enable_prefilter: bool,
    depth_relevant_max_m: float,
    crop_half_pixels: int,
) -> Dict[str, float]:
    intr = CameraIntrinsics(
        fx=615.0 * (width / 640.0),
        fy=615.0 * (height / 480.0),
        cx=0.5 * (width - 1),
        cy=0.5 * (height - 1),
        width=width,
        height=height,
    )
    depth = np.zeros((height, width), dtype=np.float64)
    detections: List[Detection2D] = []
    side = max(4, min(width, height) // 20)
    for idx in range(n_det):
        u_center = int(np.clip(intr.cx + ((idx % 5) - 2) * max(side, 1), 0, width - 1))
        v_center = int(np.clip(intr.cy + ((idx // 5) - 2) * max(side, 1), 0, height - 1))
        u0 = max(0, u_center - side // 2)
        u1 = min(width - 1, u_center + side // 2)
        v0 = max(0, v_center - side // 2)
        v1 = min(height - 1, v_center + side // 2)
        depth[v0:v1 + 1, u0:u1 + 1] = 0.35 + 0.01 * (idx % 7)
        detections.append(Detection2D("occupied", (u0, v0, u1, v1), source_id=f"det_{iteration}_{idx}"))
    active_cap = None if int(max_active_detections) <= 0 else int(max_active_detections)
    prefilter_drop_ratio = 0.0
    if bool(enable_prefilter):
        _, depth, prefilter_stats = apply_speed_prefilter(
            None,
            depth,
            max_depth_m=float(depth_relevant_max_m),
            crop_half_pixels=int(crop_half_pixels),
        )
        prefilter_drop_ratio = float(prefilter_stats.get("speed_prefilter_total_drop_ratio", 0.0))
    adapter = PerceptionToMapAdapter(
        max_points_per_detection=max_points,
        max_active_detections=active_cap,
    )
    frame = {"depth": depth, "intrinsics": intr, "timestamp_s": float(iteration)}
    T = nominal_t_base_cam(Y_PLANE_FIXED)
    T[:3, 3] = np.array([0.0, Y_PLANE_FIXED, 0.50], dtype=np.float64)
    request = adapter.from_frame(frame, detections, T_base_cam=T, source="latency_sweep", sequence_id=iteration)
    stats = dict(request.adapter_stats)
    out = {key: float(stats.get(key, 0.0)) for key in LATENCY_KEYS + COUNT_KEYS}
    out.update(
        {
            "prefilter_applied": 1.0 if bool(enable_prefilter) else 0.0,
            "prefilter_drop_ratio": float(prefilter_drop_ratio),
            "sticky_update_latency_ms": float(_measure_sticky_update_latency()),
            "path_collision_check_latency_ms": float(_measure_path_collision_latency()),
        }
    )
    return out


def _aggregate(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    groups: Dict[Tuple[int, int, int], List[Dict[str, Any]]] = {}
    for row in rows:
        key = (int(row["width"]), int(row["height"]), int(row["detection_count"]))
        groups.setdefault(key, []).append(row)
    out: List[Dict[str, Any]] = []
    for (width, height, n_det), vals in sorted(groups.items()):
        payload: Dict[str, Any] = {"width": width, "height": height, "detection_count": n_det}
        for key in LATENCY_KEYS + COUNT_KEYS + EXTENSION_KEYS:
            arr = np.asarray([float(v[key]) for v in vals], dtype=np.float64)
            payload[f"{key}_median"] = float(np.median(arr))
            payload[f"{key}_p95"] = float(np.percentile(arr, 95.0))
            payload[f"{key}_p99"] = float(np.percentile(arr, 99.0))
        c_to_f = sum(payload[f"{key}_median"] for key in LATENCY_KEYS[2:6])
        payload["stage_c_to_f_latency_ms_median"] = float(c_to_f)
        out.append(payload)
    return out


def _measure_sticky_update_latency() -> float:
    manager = StickyMapManager(params=StickyMapParams(inflation_m=0.01, n_free_frames_to_unblock=3, resolution_m=0.01))
    occ = np.zeros((32, 32), dtype=bool)
    occ[16, 16] = True
    state = manager.initialize({"occupied_mask": occ})
    free = np.zeros_like(occ)
    free[16, 16] = True
    t0 = time.perf_counter()
    manager.update(state, {"occupied_mask": np.zeros_like(occ), "free_mask": free}, np.ones_like(occ))
    return (time.perf_counter() - t0) * 1000.0


class _LatencySweepCapsule:
    def q_hits_mask(self, q_active: np.ndarray, handle: object, mask: np.ndarray, dilated_masks: object = None) -> tuple[bool, int, int]:
        q = np.asarray(q_active, dtype=np.float64).reshape(-1)
        hit = bool(q[0] > 0.5)
        return hit, int(hit), 1

    def build_dilated_masks(self, handle: object, mask: np.ndarray) -> None:
        return None


class _LatencySweepHandle:
    resolution_m = 0.01
    x0 = 0.0
    z0 = 0.0


def _measure_path_collision_latency() -> float:
    monitor = PathCollisionMonitor(_LatencySweepCapsule(), handle=_LatencySweepHandle())
    q_traj = np.zeros((16, 3), dtype=np.float64)
    q_traj[-1, 0] = 0.8
    t0 = time.perf_counter()
    monitor.check({"q_traj": q_traj}, np.zeros((8, 8), dtype=bool))
    return (time.perf_counter() - t0) * 1000.0


def _strict_latency_pass(rows: List[Dict[str, Any]]) -> bool:
    if not rows:
        return False
    worst_50 = [r for r in rows if int(r["detection_count"]) == 50]
    stage_a_ok = all(float(r["stage_a_image_prefilter_latency_ms_median"]) <= 1.0 for r in worst_50)
    large = [r for r in rows if int(r["width"]) >= 1280 and int(r["height"]) >= 720]
    stage_b_ok = all(float(r["stage_b_depth_pixels_latency_ms_median"]) <= 5.0 for r in large)
    c_to_f_ok = all(float(r["stage_c_to_f_latency_ms_median"]) <= 4.0 for r in rows)
    e2e_ok = all(float(r["adapter_latency_ms_median"]) <= 10.0 and float(r["adapter_latency_ms_p99"]) <= 33.0 for r in rows)
    return bool(stage_a_ok and stage_b_ok and c_to_f_ok and e2e_ok)


def _contractual_latency_pass(rows: List[Dict[str, Any]], max_active_detections: int) -> bool:
    if not rows or int(max_active_detections) <= 0:
        return False
    required = [
        "image_detection_count_in_median",
        "image_detection_count_active_median",
        "image_detection_count_pruned_by_topk_median",
        "max_active_detections_used_median",
        "adapter_latency_ms_median",
        "adapter_latency_ms_p99",
    ]
    if any(any(key not in row for key in required) for row in rows):
        return False
    active_bounded = all(float(row["image_detection_count_active_median"]) <= float(max_active_detections) for row in rows)
    raw_consistent = all(abs(float(row["image_detection_count_in_median"]) - float(row["detection_count"])) < 1.0e-9 for row in rows)
    cap_recorded = all(int(round(float(row["max_active_detections_used_median"]))) == int(max_active_detections) for row in rows)
    stress_rows = [row for row in rows if int(row["detection_count"]) > int(max_active_detections)]
    stress_pruned = bool(stress_rows) and all(float(row["image_detection_count_pruned_by_topk_median"]) > 0.0 for row in stress_rows)
    realtime = all(float(row["adapter_latency_ms_median"]) <= 10.0 and float(row["adapter_latency_ms_p99"]) <= 33.0 for row in rows)
    return bool(active_bounded and raw_consistent and cap_recorded and stress_pruned and realtime)


def _parse_size(text: str) -> Tuple[int, int]:
    w, h = str(text).lower().split("x", 1)
    return int(w), int(h)


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


def _write_json(path: str, data: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _write_latency_plot(path: str, rows: List[Dict[str, Any]]) -> Optional[str]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return None
    fig, ax = plt.subplots(figsize=(7.0, 4.0), dpi=150)
    for size in sorted({(r["width"], r["height"]) for r in rows}):
        vals = [r for r in rows if (r["width"], r["height"]) == size]
        vals = sorted(vals, key=lambda r: r["detection_count"])
        ax.plot(
            [r["detection_count"] for r in vals],
            [r["adapter_latency_ms_median"] for r in vals],
            marker="o",
            label=f"{size[0]}x{size[1]}",
        )
    ax.axhline(10.0, color="red", linestyle="--", linewidth=1.0, label="10 ms")
    ax.set_xlabel("detection count")
    ax.set_ylabel("median adapter latency [ms]")
    ax.set_title("PerceptionToMapAdapter synthetic latency")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


if __name__ == "__main__":
    raise SystemExit(main())
