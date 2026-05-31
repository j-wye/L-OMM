#!/usr/bin/env python3
"""Synthetic RGB-D check for PerceptionToMapAdapter.

No live camera, ROS2 topic, or segmentation model is used.  The runner creates
two synthetic detections: one on the active y-band and one far outside it.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, Optional, Sequence

import numpy as np


SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, "..", ".."))

if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)
if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(1, CONTROL_MODULE_DIR)

from control_module.constants import DEFAULT_MU_MIN, Y_PLANE_FIXED  # noqa: E402
from control_module.path_planning import PathPlanning  # noqa: E402
from map_update_layer.map_update_layer import MapUpdateLayer  # noqa: E402
from map_update_layer.perception_to_map import (  # noqa: E402
    CameraIntrinsics,
    Detection2D,
    PerceptionToMapAdapter,
    nominal_t_base_cam,
)
from map_update_layer.snapshot_validator import SnapshotValidator  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run synthetic RGB-D adapter smoke checks.")
    parser.add_argument("--map-path", default=None)
    parser.add_argument("--mu-min", type=float, default=DEFAULT_MU_MIN)
    parser.add_argument("--out", default=None)
    return parser


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "perception_to_map_adapter_checks"))
    os.makedirs(out_root, exist_ok=True)

    h, w = 120, 160
    intr = CameraIntrinsics(fx=120.0, fy=120.0, cx=0.5 * (w - 1), cy=0.5 * (h - 1), width=w, height=h)
    depth = np.zeros((h, w), dtype=np.float64)
    center_bbox = (72, 53, 87, 68)
    offband_bbox = (8, 53, 23, 68)
    depth[53:69, 72:88] = 0.35
    depth[53:69, 8:24] = 0.35
    frame = {"depth": depth, "intrinsics": intr, "timestamp_s": 0.0}
    detections = [
        Detection2D("occupied", center_bbox, source_id="center_band_box"),
        Detection2D("occupied", offband_bbox, source_id="offband_box"),
    ]

    T = nominal_t_base_cam(Y_PLANE_FIXED)
    T[:3, 3] = np.array([0.0, Y_PLANE_FIXED, 0.50], dtype=np.float64)
    adapter = PerceptionToMapAdapter()
    request = adapter.from_frame(
        frame,
        detections,
        T_base_cam=T,
        source="synthetic_rgbd_adapter_check",
        sequence_id=1,
    )

    planner = PathPlanning(map_path=args.map_path)
    handle = planner.load_map(args.map_path)
    snapshot = MapUpdateLayer().build(handle, mu_min=float(args.mu_min), request=request)
    SnapshotValidator().assert_valid(snapshot)

    summary: Dict[str, Any] = {
        "mode": "synthetic_rgbd_adapter_check",
        "explicitly_no_live_camera": True,
        "detections_in": len(detections),
        "occupied_rect_count": len(request.occupied_rects),
        "target_rect_count": len(request.target_rects),
        "unknown_rect_count": len(request.unknown_rects),
        "occluded_rect_count": len(request.occluded_rects),
        "generated_occluded_rect_count": snapshot.stats.get("generated_occluded_rect_count", 0.0),
        "total_occluded_rect_count": snapshot.stats.get("total_occluded_rect_count", len(request.occluded_rects)),
        "occupied_rects": list(request.occupied_rects),
        "target_rects": list(request.target_rects),
        "unknown_rects": list(request.unknown_rects),
        "occluded_rects": list(request.occluded_rects),
        "adapter_stats": dict(request.adapter_stats),
        "snapshot_stats": dict(snapshot.stats),
        "map_path": handle.map_path,
        "map_tag": handle.tag,
    }
    with open(os.path.join(out_root, "perception_to_map_adapter_summary.json"), "w", encoding="utf-8") as f:
        json.dump(_jsonable(summary), f, indent=2, ensure_ascii=False)
    print(
        "[perception_to_map_adapter_checks] "
        f"rects={summary['occupied_rect_count']} "
        f"snapshot_valid={float(snapshot.stats.get('snapshot_valid', 0.0)) >= 1.0} "
        f"out={out_root}"
    )
    return 0


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
