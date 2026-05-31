#!/usr/bin/env python3
"""Fixed-parameter comparison between distance A* and manipulability-cost A*.

This script is intentionally separate from tuning.  It assumes w and kappa are
already fixed, then compares the resulting path skeleton and DLS execution
metrics against the distance-only baseline over the selected height maps.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_MODULE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "control_module"))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))

if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(0, CONTROL_MODULE_DIR)

from constants import (  # noqa: E402
    DEFAULT_MANIP_KAPPA,
    DEFAULT_MANIP_MU_SAFE,
    DEFAULT_MANIP_MU_SAFE_PERCENTILE,
    DEFAULT_MANIP_WEIGHT,
    DEFAULT_MAP_UPDATE_HZ,
    DEFAULT_MU_MIN,
    DEFAULT_OBSTACLE_INFLATION_M,
)
from control_module import ControlModule  # noqa: E402
from mission_request import MissionRequest  # noqa: E402
from path_planning import PathPlanning  # noqa: E402
from plan_request import PlanRequest  # noqa: E402


@dataclass(frozen=True)
class MapSpec:
    tag: str
    path: str
    height: float
    target_y: float


def _default_map_dir() -> str:
    return os.path.join(PROJECT_ROOT, "map", "map_test", "10mm")


def _read_map_spec(map_path: str) -> MapSpec:
    path = os.path.abspath(map_path)
    base = os.path.basename(path)
    if not (base.startswith("map_") and base.endswith(".npy")):
        raise ValueError(f"map file must be named map_<tag>.npy, got {base}")
    tag = base[len("map_"):-len(".npy")]
    meta_path = os.path.join(os.path.dirname(path), f"meta_{tag}.json")
    with open(meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)
    return MapSpec(
        tag=tag,
        path=path,
        height=float(meta.get("target_height", meta.get("target_z", float("nan")))),
        target_y=float(meta.get("target_y_fixed_m", meta.get("target_y", float("nan")))),
    )


def _discover_maps(args: argparse.Namespace) -> List[MapSpec]:
    if args.map_path:
        paths = [os.path.abspath(args.map_path)]
    else:
        map_dir = os.path.abspath(args.map_dir or _default_map_dir())
        paths = sorted(
            os.path.join(map_dir, name)
            for name in os.listdir(map_dir)
            if name.startswith(args.map_prefix) and name.endswith(".npy")
        )
    specs = [_read_map_spec(p) for p in paths]
    specs.sort(key=lambda m: (m.height, m.tag))
    if args.height_min is not None:
        specs = [m for m in specs if m.height >= float(args.height_min)]
    if args.height_max is not None:
        specs = [m for m in specs if m.height <= float(args.height_max)]
    if int(args.height_limit) > 0:
        specs = specs[:int(args.height_limit)]
    if not specs:
        raise RuntimeError("No map files selected for comparison.")
    return specs


def _metric_stats(rows: Sequence[Dict[str, Any]], key: str) -> Dict[str, Any]:
    vals = np.asarray(
        [
            float(r[key])
            for r in rows
            if bool(r.get("valid_pair")) and math.isfinite(float(r.get(key, float("nan"))))
        ],
        dtype=np.float64,
    )
    if vals.size == 0:
        return {"mean": float("nan"), "min": float("nan"), "max": float("nan"), "lt1": 0, "le1": 0, "n": 0}
    return {
        "mean": float(np.mean(vals)),
        "min": float(np.min(vals)),
        "max": float(np.max(vals)),
        "lt1": int(np.sum(vals < 1.0)),
        "le1": int(np.sum(vals <= 1.0)),
        "n": int(vals.size),
    }


def _write_csv(path: str, rows: Sequence[Dict[str, Any]]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not rows:
        with open(path, "w", encoding="utf-8", newline="") as f:
            f.write("")
        return
    keys: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in keys:
                keys.append(key)
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _write_json(path: str, data: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _joint_value(control: Dict[str, Any], index: int, label: str) -> Optional[float]:
    named = control.get(f"L_q_{label}")
    if named is not None:
        return float(named)
    vals = control.get("L_q_j") or []
    if isinstance(vals, (list, tuple)) and index < len(vals) and vals[index] is not None:
        return float(vals[index])
    return None


def _case_metric(summary: Dict[str, Any]) -> Dict[str, Any]:
    case = summary["case"]
    plan = case["plan"]
    control = case["control"]
    policy = case["candidate_policy"]
    return {
        "accepted": bool(policy["candidate_accepted"]),
        "reject_reason": str(policy["reject_reason"]),
        "found": bool(plan["found"]),
        "valid_grasp": bool(plan["valid_grasp"]),
        "collision_free": bool(plan["collision_free"]),
        "path_length_m": float(plan["path_length_m"]),
        "nodes_expanded": int(plan["nodes_expanded"]),
        "planning_time_ms": float(plan["planning_time_ms"]),
        "waypoint_count": int(plan["waypoint_count"]),
        "L_q": None if control.get("L_q") is None else float(control["L_q"]),
        "L_q_j_max": None if control.get("L_q_j_max") is None else float(control["L_q_j_max"]),
        "L_q_q2": _joint_value(control, 0, "q2"),
        "L_q_q3": _joint_value(control, 1, "q3"),
        "L_q_q5": _joint_value(control, 2, "q5"),
        "rho_ref": None if control.get("rho_ref") is None else float(control["rho_ref"]),
        "success_basic": bool(control.get("success_basic", False)),
        "success_strong": bool(control.get("success_strong", False)),
        "final_pos_err_m": control.get("final_pos_err_m"),
        "final_rot_err_deg": control.get("final_rot_err_deg"),
    }


def _finite_positive(value: Any) -> bool:
    try:
        x = float(value)
    except Exception:
        return False
    return math.isfinite(x) and x > 0.0


def _valid_pair(base: Dict[str, Any], cand: Dict[str, Any]) -> bool:
    if not (base["accepted"] and cand["accepted"]):
        return False
    keys = ("path_length_m", "L_q", "L_q_j_max", "L_q_q2", "L_q_q3", "L_q_q5", "rho_ref")
    return all(_finite_positive(base.get(k)) and _finite_positive(cand.get(k)) for k in keys)


def _ratio_row(spec: MapSpec,
               base_metric: Dict[str, Any],
               cand_metric: Dict[str, Any],
               path_equal: bool,
               base_cells: Sequence[Tuple[int, int]],
               cand_cells: Sequence[Tuple[int, int]]) -> Dict[str, Any]:
    valid = _valid_pair(base_metric, cand_metric)
    row: Dict[str, Any] = {
        "tag": spec.tag,
        "height": float(spec.height),
        "target_y": float(spec.target_y),
        "path_equal": bool(path_equal),
        "distance_waypoint_count": int(len(base_cells)),
        "manip_waypoint_count": int(len(cand_cells)),
        "valid_pair": bool(valid),
        "distance_accepted": bool(base_metric["accepted"]),
        "manip_accepted": bool(cand_metric["accepted"]),
        "distance_reject": base_metric["reject_reason"],
        "manip_reject": cand_metric["reject_reason"],
    }
    for prefix, metric in (("distance", base_metric), ("manip", cand_metric)):
        for key in (
            "path_length_m", "nodes_expanded", "planning_time_ms",
            "L_q", "L_q_j_max", "L_q_q2", "L_q_q3", "L_q_q5",
            "rho_ref", "success_basic", "success_strong",
            "final_pos_err_m", "final_rot_err_deg",
        ):
            row[f"{prefix}_{key}"] = metric.get(key)
    if valid:
        row.update({
            "J_a": cand_metric["L_q"] / base_metric["L_q"],
            "J_b": cand_metric["L_q_j_max"] / base_metric["L_q_j_max"],
            "J_c": cand_metric["rho_ref"] / base_metric["rho_ref"],
            "J_q2": cand_metric["L_q_q2"] / base_metric["L_q_q2"],
            "J_q3": cand_metric["L_q_q3"] / base_metric["L_q_q3"],
            "J_q5": cand_metric["L_q_q5"] / base_metric["L_q_q5"],
            "path_length_ratio": cand_metric["path_length_m"] / base_metric["path_length_m"],
            "nodes_ratio": cand_metric["nodes_expanded"] / max(base_metric["nodes_expanded"], 1),
        })
    else:
        row.update({
            "J_a": float("nan"),
            "J_b": float("nan"),
            "J_c": float("nan"),
            "J_q2": float("nan"),
            "J_q3": float("nan"),
            "J_q5": float("nan"),
            "path_length_ratio": float("nan"),
            "nodes_ratio": float("nan"),
        })
    return row


def _mission(spec: MapSpec, cost_mode: str, args: argparse.Namespace) -> MissionRequest:
    return MissionRequest(
        scenario="clear",
        map_path=spec.path,
        obstacles=(),
        obstacle_inflation_m=float(args.inflation_m),
        map_update_hz=DEFAULT_MAP_UPDATE_HZ,
        mu_min=float(args.mu_min),
        cost_mode=cost_mode,
        manip_weight=float(args.w),
        manip_kappa=float(args.kappa),
        manip_mu_safe=float(args.mu_safe),
        manip_mu_safe_percentile=float(args.mu_safe_percentile),
        reference_backend="c2_quintic",
        scheduler_mode="path_following",
        dt=float(args.dt),
        v_ref=float(args.v_ref),
        terminal_hold_s=float(args.terminal_hold_s),
        gap_target_m=float(args.gap_target_m),
        terminal_gap_eps_m=float(args.gap_target_m),
        make_plots=False,
        write_artifacts=False,
    )


def _plan_request(spec: MapSpec, cost_mode: str, args: argparse.Namespace) -> PlanRequest:
    return PlanRequest(
        map_path=spec.path,
        obstacles=(),
        obstacle_inflation_m=float(args.inflation_m),
        map_update_hz=DEFAULT_MAP_UPDATE_HZ,
        mu_min=float(args.mu_min),
        cost_mode=cost_mode,
        manip_weight=float(args.w),
        manip_kappa=float(args.kappa),
        manip_mu_safe=float(args.mu_safe),
        manip_mu_safe_percentile=float(args.mu_safe_percentile),
    )


def _compare_one(spec: MapSpec, args: argparse.Namespace) -> Tuple[Dict[str, Any], Any, Any]:
    planner = PathPlanning()
    base_plan = planner.plan(_plan_request(spec, "distance", args))
    cand_plan = planner.plan(_plan_request(spec, "manipulability", args))
    base_cells = list(base_plan.cells)
    cand_cells = list(cand_plan.cells)
    path_equal = base_cells == cand_cells

    module = ControlModule(output_root=None)
    base_metric = _case_metric(module.run(_mission(spec, "distance", args)))
    cand_metric = _case_metric(module.run(_mission(spec, "manipulability", args)))
    row = _ratio_row(spec, base_metric, cand_metric, path_equal, base_cells, cand_cells)
    return row, base_plan, cand_plan


def _plot_single_height(path: str, spec: MapSpec, base_plan: Any, cand_plan: Any, row: Dict[str, Any], args: argparse.Namespace) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    handle = base_plan.map_handle
    extent = [
        handle.x0,
        handle.x0 + (handle.shape[0] - 1) * handle.resolution_m,
        handle.z0,
        handle.z0 + (handle.shape[1] - 1) * handle.resolution_m,
    ]
    width = max(extent[1] - extent[0], 1.0e-9)
    height = max(extent[3] - extent[2], 1.0e-9)
    fig, (ax, axm) = plt.subplots(
        1,
        2,
        figsize=(10.5, max(5.8, 5.0 * height / width)),
        dpi=140,
        gridspec_kw={"width_ratios": [1.05, 1.0]},
    )
    img = np.ma.masked_where(~base_plan.feasible_mask.T, handle.mu_grid.T)
    ax.imshow(img, origin="lower", extent=extent, aspect="equal", cmap="viridis")
    if base_plan.xz.shape[0]:
        ax.plot(base_plan.xz[:, 0], base_plan.xz[:, 1], "w--", lw=2.0, label="distance")
    if cand_plan.xz.shape[0]:
        ax.plot(cand_plan.xz[:, 0], cand_plan.xz[:, 1], color="tab:red", lw=1.8, label=f"manip w={args.w:g}, k={args.kappa:g}")
    ax.scatter([base_plan.start_xz[0]], [base_plan.start_xz[1]], c="lime", s=24, zorder=5, label="start")
    ax.scatter([base_plan.goal_xz[0]], [base_plan.goal_xz[1]], c="cyan", s=24, zorder=5, label="goal")
    for rect in base_plan.obstacle_rects:
        xl, zt, xr, zb = rect
        ax.add_patch(Rectangle((xl, zb), xr - xl, zt - zb, fill=False, edgecolor="orange", lw=1.5))
    same_text = "same path" if row["path_equal"] else "different path"
    ax.set_title(f"{spec.tag}: {same_text}")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.legend(loc="best", fontsize=7)

    def pair_label(name: str, base_key: str, cand_key: str) -> str:
        b = float(row.get(base_key, float("nan")))
        c = float(row.get(cand_key, float("nan")))
        if math.isfinite(b) and math.isfinite(c):
            return f"{name}\n{b:.3f}->{c:.3f}"
        return name

    labels = [
        pair_label("L_q", "distance_L_q", "manip_L_q"),
        pair_label("rho", "distance_rho_ref", "manip_rho_ref"),
        pair_label("q2", "distance_L_q_q2", "manip_L_q_q2"),
        pair_label("q3", "distance_L_q_q3", "manip_L_q_q3"),
        pair_label("q5", "distance_L_q_q5", "manip_L_q_q5"),
        pair_label("path", "distance_path_length_m", "manip_path_length_m"),
    ]
    values = np.asarray([float(row.get(k, float("nan"))) for k in ("J_a", "J_c", "J_q2", "J_q3", "J_q5", "path_length_ratio")])
    y = np.arange(len(labels))
    colors = []
    for value in values:
        if not math.isfinite(float(value)):
            colors.append("0.7")
        elif float(value) < 0.995:
            colors.append("tab:green")
        elif float(value) > 1.005:
            colors.append("tab:red")
        else:
            colors.append("0.55")
    axm.barh(y, values, color=colors, alpha=0.82)
    axm.axvline(1.0, color="k", linestyle="--", linewidth=1.0)
    axm.set_yticks(y)
    axm.set_yticklabels(labels)
    axm.invert_yaxis()
    finite = values[np.isfinite(values)]
    if finite.size:
        axm.set_xlim(min(0.70, float(np.min(finite)) - 0.03), max(1.08, float(np.max(finite)) + 0.03))
    axm.set_xlabel("manipulability / distance")
    axm.set_title("execution metric ratios")
    axm.grid(True, axis="x", alpha=0.3)
    for yi, value in zip(y, values):
        if math.isfinite(float(value)):
            axm.text(float(value), yi, f" {float(value):.3f}", va="center", fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _plot_summary(root: str, rows: Sequence[Dict[str, Any]], args: argparse.Namespace) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not rows:
        return
    img_root = os.path.join(root, "img")
    os.makedirs(img_root, exist_ok=True)
    rows = sorted(rows, key=lambda r: float(r["height"]))
    h = np.asarray([float(r["height"]) for r in rows], dtype=np.float64)

    def arr(key: str) -> np.ndarray:
        return np.asarray([float(r.get(key, float("nan"))) for r in rows], dtype=np.float64)

    fig, axes = plt.subplots(3, 1, figsize=(8.0, 8.5), dpi=140, sharex=True)
    axes[0].plot(h, arr("distance_L_q"), "k--", label="distance")
    axes[0].plot(h, arr("manip_L_q"), "tab:blue", label="manipulability")
    axes[0].set_ylabel("L_q")
    axes[0].legend(loc="best")
    axes[1].plot(h, arr("distance_rho_ref"), "k--", label="distance")
    axes[1].plot(h, arr("manip_rho_ref"), "tab:blue", label="manipulability")
    axes[1].set_ylabel("rho")
    axes[2].plot(h, arr("distance_path_length_m"), "k--", label="distance")
    axes[2].plot(h, arr("manip_path_length_m"), "tab:blue", label="manipulability")
    axes[2].set_ylabel("path length [m]")
    axes[2].set_xlabel("target height [m]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(img_root, "height_metric_values.png"))
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.0, 4.2), dpi=140)
    for key, label in (("J_a", "L_q"), ("J_b", "max joint"), ("J_c", "rho")):
        ax.plot(h, arr(key), label=label)
    ax.axhline(1.0, color="k", linestyle="--", linewidth=1.0)
    ax.set_xlabel("target height [m]")
    ax.set_ylabel("manipulability / distance")
    ax.set_title(f"Fixed cost metric ratios, w={args.w:g}, kappa={args.kappa:g}")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(os.path.join(img_root, "height_metric_ratios.png"))
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.0, 4.2), dpi=140)
    for key, label in (("J_q2", "q2"), ("J_q3", "q3"), ("J_q5", "q5")):
        ax.plot(h, arr(key), label=label)
    ax.axhline(1.0, color="k", linestyle="--", linewidth=1.0)
    ax.set_xlabel("target height [m]")
    ax.set_ylabel("joint travel ratio")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(os.path.join(img_root, "height_joint_ratios.png"))
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.0, 4.2), dpi=140)
    ax.plot(h, arr("path_length_ratio"), label="path length")
    ax.plot(h, arr("nodes_ratio"), label="expanded nodes")
    ax.axhline(1.0, color="k", linestyle="--", linewidth=1.0)
    ax.set_xlabel("target height [m]")
    ax.set_ylabel("manipulability / distance")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(os.path.join(img_root, "height_path_and_nodes_ratios.png"))
    plt.close(fig)


def _build_report(rows: Sequence[Dict[str, Any]], args: argparse.Namespace) -> Dict[str, Any]:
    same = [r for r in rows if bool(r["path_equal"])]
    different = [r for r in rows if not bool(r["path_equal"])]
    valid_different = [r for r in different if bool(r["valid_pair"])]
    return {
        "candidate": {"cost_mode": "manipulability", "w": float(args.w), "kappa": float(args.kappa)},
        "baseline": {"cost_mode": "distance"},
        "height_count": int(len(rows)),
        "path_equal_count": int(len(same)),
        "path_different_count": int(len(different)),
        "metric_population": "path_different_only",
        "path_equal_heights": [float(r["height"]) for r in same],
        "path_different_heights": [float(r["height"]) for r in different],
        "valid_pair_count_all": int(sum(1 for r in rows if bool(r["valid_pair"]))),
        "valid_pair_count_different": int(len(valid_different)),
        "metrics": {
            "J_a": _metric_stats(different, "J_a"),
            "J_b": _metric_stats(different, "J_b"),
            "J_c": _metric_stats(different, "J_c"),
            "J_q2": _metric_stats(different, "J_q2"),
            "J_q3": _metric_stats(different, "J_q3"),
            "J_q5": _metric_stats(different, "J_q5"),
            "path_length_ratio": _metric_stats(different, "path_length_ratio"),
            "nodes_ratio": _metric_stats(different, "nodes_ratio"),
        },
    }


def run(args: argparse.Namespace) -> int:
    maps = _discover_maps(args)
    root = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "compare"))
    if os.path.isdir(root) and not args.keep_existing:
        shutil.rmtree(root)
    os.makedirs(root, exist_ok=True)
    overlay_root = os.path.join(root, "img", "height_path_overlays")
    os.makedirs(overlay_root, exist_ok=True)

    print(f"[compare] output={root}")
    print(f"[compare] maps={len(maps)} w={args.w:g} kappa={args.kappa:g}")
    rows: List[Dict[str, Any]] = []
    different_rows: List[Dict[str, Any]] = []
    for i, spec in enumerate(maps, 1):
        row, base_plan, cand_plan = _compare_one(spec, args)
        rows.append(row)
        if not bool(row["path_equal"]):
            different_rows.append(row)
        if args.save_height_plots and not bool(row["path_equal"]):
            _plot_single_height(os.path.join(overlay_root, f"{spec.tag}.png"), spec, base_plan, cand_plan, row, args)
        if i == 1 or i == len(maps) or i % 10 == 0:
            print(f"[compare] completed {i}/{len(maps)}")

    _write_csv(os.path.join(root, "path_status_all_heights.csv"), rows)
    _write_csv(os.path.join(root, "height_comparison.csv"), different_rows)
    report = _build_report(rows, args)
    _write_json(os.path.join(root, "comparison_report.json"), report)
    _plot_summary(root, different_rows, args)
    print(
        f"[compare] same_path={report['path_equal_count']} "
        f"different_path={report['path_different_count']} "
        f"valid_different={report['valid_pair_count_different']}/{report['path_different_count']}"
    )
    print(
        f"[compare] mean Ja={report['metrics']['J_a']['mean']:.4f} "
        f"Jb={report['metrics']['J_b']['mean']:.4f} "
        f"Jc={report['metrics']['J_c']['mean']:.4f}"
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Compare fixed manipulability cost against distance-only A* over height maps.")
    p.add_argument("--map-path", default=None)
    p.add_argument("--map-dir", default=None)
    p.add_argument("--map-prefix", default="map_0.047y_")
    p.add_argument("--height-min", type=float, default=None)
    p.add_argument("--height-max", type=float, default=None)
    p.add_argument("--height-limit", type=int, default=0)
    p.add_argument("--out", default=None)
    p.add_argument("--keep-existing", action="store_true")
    p.add_argument("--w", type=float, default=DEFAULT_MANIP_WEIGHT)
    p.add_argument("--kappa", type=float, default=DEFAULT_MANIP_KAPPA)
    p.add_argument("--mu-min", type=float, default=DEFAULT_MU_MIN)
    p.add_argument("--inflation-m", type=float, default=DEFAULT_OBSTACLE_INFLATION_M)
    p.add_argument("--mu-safe", type=float, default=DEFAULT_MANIP_MU_SAFE)
    p.add_argument("--mu-safe-percentile", type=float, default=DEFAULT_MANIP_MU_SAFE_PERCENTILE)
    p.add_argument("--dt", type=float, default=0.01)
    p.add_argument("--v-ref", type=float, default=0.04)
    p.add_argument("--terminal-hold-s", type=float, default=1.0)
    p.add_argument("--gap-target-m", type=float, default=0.01)
    p.add_argument("--save-height-plots", action="store_true", default=True)
    p.add_argument("--no-height-plots", dest="save_height_plots", action="store_false")
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
