#!/usr/bin/env python3
"""Synthetic checks for depth-derived active-map geometry evidence.

These checks deliberately use stored arrays only.  They do not subscribe to
ROS2 topics, open a camera, or command robot hardware.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
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
        description="Run synthetic depth geometry evidence checks without live camera or robot motion."
    )
    parser.add_argument("--out", default=None)
    return parser


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "depth_geometry_synthetic"))
    os.makedirs(out_root, exist_ok=True)

    handle = _synthetic_handle()
    T = nominal_t_base_cam(Y_PLANE_FIXED)
    bridge = CameraFrameMapUpdateBridge()

    cases = [
        _case_target_semantic_table_depth_only,
        _case_nearby_object_depth_only,
        _case_outside_task_band_ignored,
        _case_beyond_task_horizon_ignored,
        _case_invalid_depth_not_free,
        _case_single_cell_jitter_ignored,
        _case_semantic_depth_union,
    ]

    rows: List[Dict[str, Any]] = []
    summaries: Dict[str, Dict[str, Any]] = {}
    for factory in cases:
        case = factory()
        result = bridge.process(
            case["frame"],
            case["evidence"],
            handle,
            mu_min=DEFAULT_MU_MIN,
            T_base_cam=T,
            sequence_id=int(case["sequence_id"]),
        )
        stats = dict(result.stats)
        adapter_stats = dict(result.request.adapter_stats)
        depth_stats = {
            key: adapter_stats.get(key)
            for key in adapter_stats
            if str(key).startswith("depth_geometry_")
        }
        metrics = _evaluate_case(case["name"], result, adapter_stats)
        row = {
            "case": case["name"],
            "pass": bool(metrics["pass"]),
            "snapshot_valid": float(result.validation.get("snapshot_valid", 0.0)),
            "occupied_rect_count": len(result.request.occupied_rects),
            "target_rect_count": len(result.request.target_rects),
            "unknown_rect_count": len(result.request.unknown_rects),
            "depth_geometry_occupied_rect_count": int(adapter_stats.get("depth_geometry_occupied_rect_count", 0)),
            "depth_geometry_unknown_rect_count": int(adapter_stats.get("depth_geometry_unknown_rect_count", 0)),
            "depth_geometry_added_occupied_cells": int(adapter_stats.get("depth_geometry_added_occupied_cells", 0)),
            "depth_geometry_added_unknown_cells": int(adapter_stats.get("depth_geometry_added_unknown_cells", 0)),
            "depth_geometry_filtered_outside_band_count": int(adapter_stats.get("depth_geometry_filtered_outside_band_count", 0)),
            "depth_geometry_filtered_outside_depth_count": int(adapter_stats.get("depth_geometry_filtered_outside_depth_count", 0)),
            "depth_geometry_filtered_small_cluster_count": int(adapter_stats.get("depth_geometry_filtered_small_cluster_count", 0)),
        }
        row.update(metrics)
        rows.append(row)
        summaries[case["name"]] = {
            "case": case["name"],
            "metrics": row,
            "depth_geometry_stats": depth_stats,
            "snapshot_stats": _jsonable(stats),
            "request_rects": {
                "occupied": _jsonable(result.request.occupied_rects),
                "target": _jsonable(result.request.target_rects),
                "unknown": _jsonable(result.request.unknown_rects),
            },
        }

    aggregate = {
        "mode": "depth_geometry_synthetic",
        "explicitly_no_live_camera": True,
        "explicitly_no_ros2_topic": True,
        "explicitly_no_physical_robot_execution": True,
        "all_cases_pass": bool(all(bool(row["pass"]) for row in rows)),
        "snapshot_valid_all": bool(all(float(row["snapshot_valid"]) == 1.0 for row in rows)),
        "target_semantic_preserved": _case_pass(rows, "target_semantic_table_depth_only", "target_semantic_preserved"),
        "depth_only_support_surface_blocked": _case_pass(rows, "target_semantic_table_depth_only", "depth_only_support_surface_blocked"),
        "depth_only_nearby_geometry_blocked": _case_pass(rows, "nearby_object_depth_only", "depth_only_nearby_geometry_blocked"),
        "outside_task_band_ignored": _case_pass(rows, "outside_task_band_ignored", "outside_task_band_ignored"),
        "beyond_depth_horizon_ignored": _case_pass(rows, "beyond_task_horizon_ignored", "beyond_depth_horizon_ignored"),
        "invalid_depth_not_free": _case_pass(rows, "invalid_depth_not_free", "invalid_depth_not_free"),
        "single_cell_jitter_ignored": _case_pass(rows, "single_cell_jitter_ignored", "single_cell_jitter_ignored"),
        "semantic_depth_union_valid": _case_pass(rows, "semantic_depth_union", "semantic_depth_union_valid"),
        "cases": summaries,
        "artifact_paths": {
            "summary": os.path.join(out_root, "summary.json"),
            "case_metrics": os.path.join(out_root, "case_metrics.csv"),
        },
    }

    _write_csv(os.path.join(out_root, "case_metrics.csv"), rows)
    _write_json(os.path.join(out_root, "summary.json"), aggregate)
    print(f"[depth_geometry_synthetic] all_cases_pass={aggregate['all_cases_pass']} out={out_root}")
    return 0 if aggregate["all_cases_pass"] else 1


def _synthetic_handle() -> MapHandle:
    shape = (110, 120)
    return MapHandle(
        map_path="synthetic_depth_geometry",
        meta_path="synthetic_depth_geometry_meta",
        mu_grid=np.full(shape, 0.05, dtype=np.float64),
        pitch_grid=np.zeros(shape, dtype=np.float64),
        q_grid=None,
        meta={"mu_min_runtime": DEFAULT_MU_MIN},
        resolution_m=0.01,
        x0=0.0,
        z0=-0.55,
        target_y=Y_PLANE_FIXED,
        tag="depth_geometry_synthetic",
    )


def _base_frame() -> SimpleNamespace:
    h, w = 96, 320
    K = np.array(
        [
            [120.0, 0.0, 0.5 * (w - 1)],
            [0.0, 120.0, 0.5 * (h - 1)],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return SimpleNamespace(
        rgb=np.zeros((h, w, 3), dtype=np.uint8),
        depth=np.zeros((h, w), dtype=np.float64),
        intrinsics=K,
        timestamp=10.0,
        received_timestamp=10.0,
    )


def _put_rect_depth(frame: SimpleNamespace, bbox: Tuple[int, int, int, int], depth_m: float) -> None:
    u0, v0, u1, v1 = bbox
    frame.depth[v0:v1 + 1, u0:u1 + 1] = float(depth_m)


def _case_target_semantic_table_depth_only() -> Dict[str, Any]:
    frame = _base_frame()
    _put_rect_depth(frame, (155, 38, 164, 48), 0.34)
    _put_rect_depth(frame, (146, 54, 172, 62), 0.36)
    return {
        "name": "target_semantic_table_depth_only",
        "sequence_id": 1,
        "frame": frame,
        "evidence": [Detection2D("target", (155, 38, 164, 48), source_id="target")],
    }


def _case_nearby_object_depth_only() -> Dict[str, Any]:
    frame = _base_frame()
    _put_rect_depth(frame, (166, 40, 174, 52), 0.38)
    return {
        "name": "nearby_object_depth_only",
        "sequence_id": 2,
        "frame": frame,
        "evidence": [],
    }


def _case_outside_task_band_ignored() -> Dict[str, Any]:
    frame = _base_frame()
    _put_rect_depth(frame, (2, 40, 8, 52), 0.35)
    return {
        "name": "outside_task_band_ignored",
        "sequence_id": 3,
        "frame": frame,
        "evidence": [],
    }


def _case_beyond_task_horizon_ignored() -> Dict[str, Any]:
    frame = _base_frame()
    _put_rect_depth(frame, (154, 40, 166, 52), 1.20)
    return {
        "name": "beyond_task_horizon_ignored",
        "sequence_id": 4,
        "frame": frame,
        "evidence": [],
    }


def _case_invalid_depth_not_free() -> Dict[str, Any]:
    frame = _base_frame()
    frame.depth[34:64, 146:174] = np.nan
    return {
        "name": "invalid_depth_not_free",
        "sequence_id": 5,
        "frame": frame,
        "evidence": [],
    }


def _case_single_cell_jitter_ignored() -> Dict[str, Any]:
    frame = _base_frame()
    _put_rect_depth(frame, (160, 48, 160, 48), 0.34)
    return {
        "name": "single_cell_jitter_ignored",
        "sequence_id": 6,
        "frame": frame,
        "evidence": [],
    }


def _case_semantic_depth_union() -> Dict[str, Any]:
    frame = _base_frame()
    _put_rect_depth(frame, (155, 38, 164, 48), 0.34)
    _put_rect_depth(frame, (140, 40, 148, 52), 0.37)
    _put_rect_depth(frame, (170, 54, 180, 62), 0.39)
    return {
        "name": "semantic_depth_union",
        "sequence_id": 7,
        "frame": frame,
        "evidence": [
            Detection2D("target", (155, 38, 164, 48), source_id="target"),
            Detection2D("occupied", (140, 40, 148, 52), source_id="semantic_obstacle"),
        ],
    }


def _evaluate_case(name: str, result: Any, stats: Dict[str, Any]) -> Dict[str, Any]:
    snapshot_valid = float(result.validation.get("snapshot_valid", 0.0)) == 1.0
    target_preserved = len(result.request.target_rects) > 0 and float(result.snapshot.stats.get("target_cells", 0.0)) > 0.0
    depth_occ = int(stats.get("depth_geometry_occupied_rect_count", 0))
    depth_unknown = int(stats.get("depth_geometry_unknown_rect_count", 0))
    added_occ = int(stats.get("depth_geometry_added_occupied_cells", 0))
    added_unknown = int(stats.get("depth_geometry_added_unknown_cells", 0))
    filtered_band = int(stats.get("depth_geometry_filtered_outside_band_count", 0))
    filtered_depth = int(stats.get("depth_geometry_filtered_outside_depth_count", 0))
    filtered_small = int(stats.get("depth_geometry_filtered_small_cluster_count", 0))
    semantic_occ = int(stats.get("detections_blocked", 0))

    out: Dict[str, Any] = {
        "target_semantic_preserved": False,
        "depth_only_support_surface_blocked": False,
        "depth_only_nearby_geometry_blocked": False,
        "outside_task_band_ignored": False,
        "beyond_depth_horizon_ignored": False,
        "invalid_depth_not_free": False,
        "single_cell_jitter_ignored": False,
        "semantic_depth_union_valid": False,
    }
    if name == "target_semantic_table_depth_only":
        out["target_semantic_preserved"] = bool(target_preserved)
        out["depth_only_support_surface_blocked"] = bool(depth_occ > 0 and added_occ > 0)
        out["pass"] = bool(snapshot_valid and out["target_semantic_preserved"] and out["depth_only_support_surface_blocked"])
    elif name == "nearby_object_depth_only":
        out["depth_only_nearby_geometry_blocked"] = bool(depth_occ > 0 and added_occ > 0)
        out["pass"] = bool(snapshot_valid and out["depth_only_nearby_geometry_blocked"])
    elif name == "outside_task_band_ignored":
        out["outside_task_band_ignored"] = bool(filtered_band > 0 and depth_occ == 0 and added_occ == 0)
        out["pass"] = bool(snapshot_valid and out["outside_task_band_ignored"])
    elif name == "beyond_task_horizon_ignored":
        out["beyond_depth_horizon_ignored"] = bool(filtered_depth > 0 and depth_occ == 0 and added_occ == 0)
        out["pass"] = bool(snapshot_valid and out["beyond_depth_horizon_ignored"])
    elif name == "invalid_depth_not_free":
        out["invalid_depth_not_free"] = bool(depth_unknown > 0 and added_unknown > 0)
        out["pass"] = bool(snapshot_valid and out["invalid_depth_not_free"])
    elif name == "single_cell_jitter_ignored":
        out["single_cell_jitter_ignored"] = bool(filtered_small > 0 and depth_occ == 0 and added_occ == 0)
        out["pass"] = bool(snapshot_valid and out["single_cell_jitter_ignored"])
    elif name == "semantic_depth_union":
        out["semantic_depth_union_valid"] = bool(target_preserved and semantic_occ >= 2 and depth_occ > 0 and added_occ > 0)
        out["pass"] = bool(snapshot_valid and out["semantic_depth_union_valid"])
    else:
        out["pass"] = False
    return out


def _case_pass(rows: List[Dict[str, Any]], case_name: str, key: str) -> bool:
    for row in rows:
        if row["case"] == case_name:
            return bool(row.get(key, False))
    return False


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


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
