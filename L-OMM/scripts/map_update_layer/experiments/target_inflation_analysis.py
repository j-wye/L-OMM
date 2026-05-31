#!/usr/bin/env python3
"""P2.3 target inflation vs EE-safe goal window sweep."""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, "..", ".."))

if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)
if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(1, CONTROL_MODULE_DIR)

from control_module.constants import (  # noqa: E402
    DEFAULT_COST_MODE,
    DEFAULT_MANIP_KAPPA,
    DEFAULT_MANIP_MU_SAFE,
    DEFAULT_MANIP_MU_SAFE_PERCENTILE,
    DEFAULT_MANIP_WEIGHT,
    DEFAULT_MU_MIN,
    X_EE_GOAL,
    X_TARGET_CONTACT,
)
from control_module.control_module import ControlModule  # noqa: E402
from control_module.mission_request import MissionRequest  # noqa: E402
from control_module.path_planning import PathPlanning  # noqa: E402
from map_update_layer.map_update_layer import MapUpdateLayer  # noqa: E402
from map_update_layer.map_update_request import MapUpdateRequest  # noqa: E402
from map_update_layer.snapshot_validator import SnapshotValidator  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep target_rect size and inflation against the EE-safe goal window. "
            "No live camera, ROS2 topic, or physical robot execution."
        )
    )
    parser.add_argument("--map-path", default=None)
    parser.add_argument("--mu-min", type=float, default=DEFAULT_MU_MIN)
    parser.add_argument("--cost-mode", choices=("distance", "manipulability", "all"), default=DEFAULT_COST_MODE)
    parser.add_argument("--w", "--manip-weight", dest="manip_weight", type=float, default=DEFAULT_MANIP_WEIGHT)
    parser.add_argument("--kappa", "--manip-kappa", dest="manip_kappa", type=float, default=DEFAULT_MANIP_KAPPA)
    parser.add_argument("--reference-backend", choices=("linear", "c2_quintic"), default="c2_quintic")
    parser.add_argument("--inflations-m", default="0.005,0.010,0.020,0.030")
    parser.add_argument("--target-x-extents-m", default="0.06,0.10,0.14,0.20")
    parser.add_argument("--target-z-extent-m", type=float, default=0.10)
    parser.add_argument("--out", default=None)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "target_inflation_analysis"))
    os.makedirs(out_root, exist_ok=True)
    modes = ["distance", "manipulability"] if args.cost_mode == "all" else [str(args.cost_mode)]
    inflations = _float_list(args.inflations_m)
    x_extents = _float_list(args.target_x_extents_m)
    rows: List[Dict[str, Any]] = []
    for mode in modes:
        rows.extend(_run_mode(args, mode, out_root, inflations, x_extents))
    summary = _summarize(rows, inflations, x_extents)
    _write_csv(os.path.join(out_root, "target_inflation_analysis.csv"), rows)
    _write_json(os.path.join(out_root, "target_inflation_analysis_summary.json"), summary)
    _write_heatmap(os.path.join(out_root, "target_inflation_validity_heatmap.png"), rows, inflations, x_extents)
    print(
        "[target_inflation_analysis] "
        f"rows={len(rows)} has_boundary={summary['has_valid_and_ungraspable']} out={out_root}"
    )
    return 0 if summary["pass"] else 1


def _run_mode(
    args: argparse.Namespace,
    mode: str,
    out_root: str,
    inflations: List[float],
    x_extents: List[float],
) -> List[Dict[str, Any]]:
    planner = PathPlanning(map_path=args.map_path)
    handle = planner.load_map(args.map_path)
    updater = MapUpdateLayer()
    validator = SnapshotValidator()
    clear_snapshot = updater.build(handle, mu_min=float(args.mu_min), request=MapUpdateRequest(source="target_inflation_clear"))
    validator.assert_valid(clear_snapshot)
    control = ControlModule(output_root=os.path.join(out_root, mode))
    meta = getattr(handle, "meta", {}) or {}
    x_goal = float(meta.get("x_opt_ee_goal_m", meta.get("optimal_x", X_EE_GOAL)))
    x_target = float(meta.get("x_target_contact_m", meta.get("x_gaze", X_TARGET_CONTACT)))
    target_z = float(meta.get("target_z", meta.get("target_height", 0.50)))
    rows: List[Dict[str, Any]] = []
    for x_extent in x_extents:
        for inflation in inflations:
            rect = (
                float(x_target - 0.5 * x_extent),
                float(target_z + 0.5 * float(args.target_z_extent_m)),
                float(x_target + 0.5 * x_extent),
                float(target_z - 0.5 * float(args.target_z_extent_m)),
            )
            request = MapUpdateRequest(
                target_rects=(rect,),
                inflation_m=float(inflation),
                sensor_inflation_m=float(inflation),
                source=f"target_inflation_x{x_extent:.3f}_infl{inflation:.3f}",
            )
            snapshot = updater.build(handle, mu_min=float(args.mu_min), request=request)
            validator.assert_valid(snapshot)
            point_dir = os.path.join(
                out_root,
                mode,
                f"x_extent_{x_extent:.3f}_inflation_{inflation:.3f}".replace(".", "p"),
            )
            mission = MissionRequest(
                scenario="changed",
                clear_snapshot=clear_snapshot,
                active_snapshot=snapshot,
                cost_mode=mode,
                manip_weight=float(args.manip_weight),
                manip_kappa=float(args.manip_kappa),
                manip_mu_safe=DEFAULT_MANIP_MU_SAFE,
                manip_mu_safe_percentile=DEFAULT_MANIP_MU_SAFE_PERCENTILE,
                reference_backend=str(args.reference_backend),
                make_plots=False,
                write_artifacts=True,
            )
            result = control.run(mission, out_dir=point_dir)
            rows.append(_row_from_result(mode, rect, x_goal, x_target, x_extent, inflation, snapshot, result))
    return rows


def _row_from_result(
    mode: str,
    rect: Tuple[float, float, float, float],
    x_goal: float,
    x_target: float,
    x_extent: float,
    inflation: float,
    snapshot: Any,
    result: Dict[str, Any],
) -> Dict[str, Any]:
    case = result.get("case", {})
    plan = case.get("plan", {})
    policy = case.get("candidate_policy", {})
    inflated_left = float(rect[0]) - float(inflation)
    return {
        "cost_mode": mode,
        "target_x_extent_m": float(x_extent),
        "target_inflation_m": float(inflation),
        "target_rect_x_left_m": float(rect[0]),
        "target_rect_x_right_m": float(rect[2]),
        "target_x_contact_m": float(x_target),
        "ee_goal_x_m": float(x_goal),
        "inflated_target_left_edge_m": float(inflated_left),
        "goal_clearance_after_inflation_m": float(inflated_left - x_goal),
        "blocked_cells": int(snapshot.stats.get("blocked_cells", 0.0)),
        "final_feasible_cells": int(snapshot.stats.get("final_feasible_cells", 0.0)),
        "plan_found": bool(plan.get("found", False)),
        "valid_grasp": bool(plan.get("valid_grasp", False)),
        "candidate_accepted": bool(policy.get("candidate_accepted", False)),
        "reject_reason": str(policy.get("reject_reason", "")),
        "invalid_reason_code": str(plan.get("invalid_reason_code", "")),
        "goal_projection_m": float(plan.get("goal_projection_m", float("nan"))),
        "projection_tolerance_m": float(plan.get("projection_tolerance_m", float("nan"))),
        "path_length_m": float(plan.get("path_length_m", 0.0)),
        "run_dir": str(result.get("run_dir", "")),
    }


def _summarize(rows: List[Dict[str, Any]], inflations: List[float], x_extents: List[float]) -> Dict[str, Any]:
    has_valid = any(bool(row["valid_grasp"]) and bool(row["plan_found"]) for row in rows)
    has_ungraspable = any(not bool(row["valid_grasp"]) or not bool(row["plan_found"]) for row in rows)
    boundary_rows = [
        row for row in rows
        if float(row["goal_clearance_after_inflation_m"]) <= float(row["projection_tolerance_m"])
    ]
    return {
        "mode": "target_inflation_vs_ee_safe_goal_window",
        "explicitly_no_live_camera": True,
        "explicitly_no_ros2_topic": True,
        "explicitly_no_physical_robot_execution": True,
        "inflation_values_m": inflations,
        "target_x_extents_m": x_extents,
        "row_count": len(rows),
        "valid_count": int(sum(1 for row in rows if bool(row["valid_grasp"]) and bool(row["plan_found"]))),
        "ungraspable_count": int(sum(1 for row in rows if not bool(row["valid_grasp"]) or not bool(row["plan_found"]))),
        "has_valid_and_ungraspable": bool(has_valid and has_ungraspable),
        "boundary_candidate_count": len(boundary_rows),
        "pass": bool(has_valid and has_ungraspable),
    }


def _float_list(text: str) -> List[float]:
    return [float(v.strip()) for v in str(text).split(",") if v.strip()]


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


def _write_heatmap(path: str, rows: List[Dict[str, Any]], inflations: List[float], x_extents: List[float]) -> Optional[str]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return None
    modes = sorted({str(row["cost_mode"]) for row in rows})
    fig, axes = plt.subplots(1, len(modes), figsize=(5.0 * len(modes), 4.0), dpi=150, squeeze=False)
    for ax, mode in zip(axes[0], modes):
        grid = np.zeros((len(x_extents), len(inflations)), dtype=float)
        for row in rows:
            if str(row["cost_mode"]) != mode:
                continue
            i = x_extents.index(float(row["target_x_extent_m"]))
            j = inflations.index(float(row["target_inflation_m"]))
            grid[i, j] = 1.0 if bool(row["valid_grasp"]) and bool(row["plan_found"]) else 0.0
        im = ax.imshow(grid, origin="lower", cmap="RdYlGn", vmin=0.0, vmax=1.0)
        ax.set_xticks(range(len(inflations)), [f"{v*1000:.0f}" for v in inflations])
        ax.set_yticks(range(len(x_extents)), [f"{v*1000:.0f}" for v in x_extents])
        ax.set_xlabel("inflation [mm]")
        ax.set_ylabel("target x extent [mm]")
        ax.set_title(mode)
        fig.colorbar(im, ax=ax)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


if __name__ == "__main__":
    raise SystemExit(main())

