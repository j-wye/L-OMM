#!/usr/bin/env python3
"""Unit checks for ReferenceSnapshotDiff and task-plane FOV masks."""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, Optional, Sequence

import numpy as np


SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, "..", ".."))

if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)
if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(1, CONTROL_MODULE_DIR)

from control_module.map_handle import MapHandle  # noqa: E402
from map_update_layer.active_map_snapshot import ActiveMapSnapshot  # noqa: E402
from map_update_layer.fov_mask import compute_task_plane_fov_mask  # noqa: E402
from map_update_layer.perception_to_map import CameraIntrinsics, nominal_t_base_cam  # noqa: E402
from map_update_layer.reference_snapshot_diff import ReferenceSnapshotDiff  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run deterministic ReferenceSnapshotDiff checks without live camera or robot execution."
    )
    parser.add_argument("--out", default=None, help="Output directory. Defaults to path/reference_snapshot_diff_checks.")
    return parser


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_dir = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "reference_snapshot_diff_checks"))
    os.makedirs(out_dir, exist_ok=True)

    handle = _synthetic_handle()
    intrinsics = CameraIntrinsics(fx=615.0, fy=615.0, cx=319.5, cy=239.5, width=640, height=480)
    T = nominal_t_base_cam(handle.target_y)
    reference_corridor = _line_xz(0.10, 0.60, 0.0, 51)

    ref_clear = _snapshot(handle, np.zeros(handle.shape, dtype=bool), source="ref_clear")
    current_new = _snapshot(handle, _rect_mask(handle, 0.30, -0.015, 0.36, 0.015), source="current_new")
    current_jitter = _snapshot(handle, _one_cell_mask(handle, 0.32, 0.0), source="current_jitter")
    ref_blocked = _snapshot(handle, _rect_mask(handle, 0.30, -0.015, 0.36, 0.015), source="ref_blocked")
    current_clear = _snapshot(handle, np.zeros(handle.shape, dtype=bool), source="current_clear")

    fov_near = compute_task_plane_fov_mask(handle, intrinsics, T, handle.target_y, 0.05, 0.35, pixel_band_half_width=100.0)
    fov_far = compute_task_plane_fov_mask(handle, intrinsics, T, handle.target_y, 0.05, 0.75, pixel_band_half_width=100.0)
    full_fov = np.ones(handle.shape, dtype=bool)
    empty_fov = np.zeros(handle.shape, dtype=bool)

    checks: Dict[str, Dict[str, object]] = {}
    checks["fov_mask_expands_with_depth"] = {
        "pass": bool(np.count_nonzero(fov_far) > np.count_nonzero(fov_near) > 0),
        "fov_near_cells": int(np.count_nonzero(fov_near)),
        "fov_far_cells": int(np.count_nonzero(fov_far)),
    }

    diff = ReferenceSnapshotDiff(
        handle=handle,
        intrinsics=intrinsics,
        y_plane=handle.target_y,
        d_min=0.05,
        d_max_task=0.75,
        pixel_band_half_width=100.0,
        tau_diff_cells=3,
        tau_cluster_cells=5,
    )
    diff.capture(ref_clear, q_act=None, T_base_cam=T, reference_corridor=reference_corridor)
    report = diff.evaluate(current_new, T, current_corridor=reference_corridor, fov_mask_override=full_fov)
    checks["new_obstacle_on_reference_corridor"] = {
        "pass": bool(report.is_corridor_violated),
        **report.as_dict(),
    }

    diff.capture(ref_clear, q_act=None, T_base_cam=T, reference_corridor=reference_corridor)
    report = diff.evaluate(current_new, T, current_corridor=reference_corridor, fov_mask_override=empty_fov)
    checks["fov_outside_change_ignored"] = {
        "pass": bool(report.new_cells_in_reference_corridor == 0 and not report.is_corridor_violated),
        **report.as_dict(),
    }

    diff.capture(ref_blocked, q_act=None, T_base_cam=T, reference_corridor=reference_corridor)
    report = diff.evaluate(current_clear, T, current_corridor=reference_corridor, fov_mask_override=full_fov)
    checks["vanished_obstacle_nontrigger"] = {
        "pass": bool(report.vanished_obstacle_cells > 0 and not report.is_corridor_violated),
        **report.as_dict(),
    }

    diff.capture(ref_clear, q_act=None, T_base_cam=T, reference_corridor=reference_corridor)
    report = diff.evaluate(current_jitter, T, current_corridor=reference_corridor, fov_mask_override=full_fov)
    checks["single_cell_jitter_ignored"] = {
        "pass": bool(report.new_obstacle_cells == 1 and not report.is_corridor_violated),
        **report.as_dict(),
    }

    summary = {
        "mode": "reference_snapshot_diff_unit_checks",
        "explicitly_no_live_camera": True,
        "explicitly_no_ros2_topic": True,
        "explicitly_no_physical_robot_execution": True,
        "all_checks_pass": bool(all(bool(item["pass"]) for item in checks.values())),
        "checks": checks,
    }
    _write_json(os.path.join(out_dir, "reference_snapshot_diff_summary.json"), summary)
    print(f"[reference_snapshot_diff_checks] all_checks_pass={summary['all_checks_pass']} out={out_dir}")
    return 0 if summary["all_checks_pass"] else 1


def _synthetic_handle() -> MapHandle:
    shape = (80, 80)
    return MapHandle(
        map_path="synthetic_reference_diff",
        meta_path="synthetic_reference_diff_meta",
        mu_grid=np.ones(shape, dtype=np.float64),
        pitch_grid=np.zeros(shape, dtype=np.float64),
        q_grid=None,
        meta={},
        resolution_m=0.01,
        x0=0.0,
        z0=-0.40,
        target_y=0.047,
        tag="reference_diff_unit",
    )


def _snapshot(handle: MapHandle, blocked: np.ndarray, *, source: str) -> ActiveMapSnapshot:
    base = np.ones(handle.shape, dtype=bool)
    blocked_mask = np.asarray(blocked, dtype=bool)
    return ActiveMapSnapshot(
        handle=handle,
        base_feasible_mask=base,
        final_active_mask=np.logical_and(base, ~blocked_mask),
        blocked_mask=blocked_mask,
        stats={"snapshot_valid": 1.0},
        layer_masks={"blocked": blocked_mask},
        source=source,
        mu_min=0.0,
    )


def _rect_mask(handle: MapHandle, x0: float, z0: float, x1: float, z1: float) -> np.ndarray:
    mask = np.zeros(handle.shape, dtype=bool)
    ix0 = max(0, int(np.floor((min(x0, x1) - handle.x0) / handle.resolution_m)))
    ix1 = min(handle.shape[0] - 1, int(np.ceil((max(x0, x1) - handle.x0) / handle.resolution_m)))
    iz0 = max(0, int(np.floor((min(z0, z1) - handle.z0) / handle.resolution_m)))
    iz1 = min(handle.shape[1] - 1, int(np.ceil((max(z0, z1) - handle.z0) / handle.resolution_m)))
    mask[ix0:ix1 + 1, iz0:iz1 + 1] = True
    return mask


def _one_cell_mask(handle: MapHandle, x: float, z: float) -> np.ndarray:
    mask = np.zeros(handle.shape, dtype=bool)
    ix = int(round((float(x) - handle.x0) / handle.resolution_m))
    iz = int(round((float(z) - handle.z0) / handle.resolution_m))
    if 0 <= ix < handle.shape[0] and 0 <= iz < handle.shape[1]:
        mask[ix, iz] = True
    return mask


def _line_xz(x0: float, x1: float, z: float, n: int) -> np.ndarray:
    xs = np.linspace(float(x0), float(x1), int(n), dtype=np.float64)
    zs = np.full(xs.shape, float(z), dtype=np.float64)
    return np.stack([xs, zs], axis=1)


def _write_json(path: str, data: Dict[str, object]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(_jsonable(data), f, indent=2, ensure_ascii=False)


def _jsonable(value):
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
