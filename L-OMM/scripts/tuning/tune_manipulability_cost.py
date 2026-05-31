#!/usr/bin/env python3
"""Grid tuning for reduced-active manipulability edge regularization."""
from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_MODULE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "control_module"))
if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(0, CONTROL_MODULE_DIR)

from constants import (
    DEFAULT_MANIP_MU_SAFE,
    DEFAULT_MANIP_MU_SAFE_PERCENTILE,
    DEFAULT_MAP_UPDATE_HZ,
    DEFAULT_MU_MIN,
    DEFAULT_OBSTACLE_INFLATION_M,
    Rect,
)
from control_module import ControlModule
from mission_request import MissionRequest


DEFAULT_W_GRID = ",".join(f"{0.05 * i:.2f}" for i in range(1, 31))
DEFAULT_KAPPA_GRID = ",".join(f"{0.5 * i:.1f}" for i in range(1, 21))


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    scenario: str
    obstacles: Tuple[Rect, ...] = ()


@dataclass(frozen=True)
class MapSpec:
    tag: str
    path: str
    height: float
    target_y: float


CLEAR_SCENARIOS: Tuple[ScenarioSpec, ...] = (
    ScenarioSpec("clear", "clear", ()),
)

OBSTACLE_SCENARIOS: Tuple[ScenarioSpec, ...] = (
    ScenarioSpec("obs_lower_left", "changed", ((0.16, 0.36, 0.26, 0.22),)),
    ScenarioSpec("obs_lower_mid", "changed", ((0.24, 0.40, 0.34, 0.24),)),
    ScenarioSpec("obs_blocking_right", "changed", ((0.34, 0.68, 0.44, 0.54),)),
)


def _scenario_set(args: argparse.Namespace) -> Tuple[ScenarioSpec, ...]:
    if bool(getattr(args, "include_obstacles", False)):
        return CLEAR_SCENARIOS + OBSTACLE_SCENARIOS
    return CLEAR_SCENARIOS


def _parse_grid(raw: str) -> List[float]:
    return [float(v.strip()) for v in raw.split(",") if v.strip()]


def _project_root() -> str:
    module_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(module_dir, "..", "..", ".."))


def _default_map_dir() -> str:
    return os.path.join(_project_root(), "map", "map_test", "10mm")


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
    if args.all_heights:
        map_dir = os.path.abspath(args.map_dir or _default_map_dir())
        paths = sorted(
            os.path.join(map_dir, name)
            for name in os.listdir(map_dir)
            if name.startswith(args.map_prefix) and name.endswith(".npy")
        )
    else:
        if args.map_path:
            paths = [os.path.abspath(args.map_path)]
        else:
            paths = [os.path.join(_default_map_dir(), "map_0.047y_0.5z.npy")]
    specs = [_read_map_spec(p) for p in paths]
    specs.sort(key=lambda m: (m.height, m.tag))
    if args.height_min is not None:
        specs = [m for m in specs if m.height >= float(args.height_min)]
    if args.height_max is not None:
        specs = [m for m in specs if m.height <= float(args.height_max)]
    if int(args.height_limit) > 0:
        specs = specs[:int(args.height_limit)]
    if not specs:
        raise RuntimeError("No map files selected for tuning.")
    return specs


def _finite_positive(v: Any) -> bool:
    try:
        x = float(v)
    except Exception:
        return False
    return math.isfinite(x) and x > 0.0


def _extract(summary: Dict[str, Any]) -> Dict[str, Any]:
    case = summary["case"]
    plan = case["plan"]
    control = case["control"]
    policy = case["candidate_policy"]
    ref = case["reference"]
    L_q_j = control.get("L_q_j") or []
    if not isinstance(L_q_j, (list, tuple)):
        L_q_j = []
    def _joint_value(index: int, label: str) -> Optional[float]:
        named = control.get(f"L_q_{label}")
        if named is not None:
            return float(named)
        if index < len(L_q_j) and L_q_j[index] is not None:
            return float(L_q_j[index])
        return None
    return {
        "found": bool(plan["found"]),
        "collision_free": bool(plan["collision_free"]),
        "accepted": bool(policy["candidate_accepted"]),
        "reject_reason": str(policy["reject_reason"]),
        "map_tag": str(plan.get("map_tag", "")),
        "map_target_height": float(plan.get("map_target_height", float("nan"))),
        "map_target_y": float(plan.get("map_target_y", float("nan"))),
        "map_x_ee_goal": float(plan.get("map_x_ee_goal", float("nan"))),
        "map_x_target_contact": float(plan.get("map_x_target_contact", float("nan"))),
        "path_length_m": float(plan["path_length_m"]),
        "g_cost": float(plan["g_cost"]),
        "nodes_expanded": int(plan["nodes_expanded"]),
        "planning_time_ms": float(plan["planning_time_ms"]),
        "waypoint_count": int(plan["waypoint_count"]),
        "mu_min_path": float(plan["path_mu_stats"].get("min", float("nan"))),
        "mu_p5_path": float(plan["path_mu_stats"].get("p5", float("nan"))),
        "mu_mean_path": float(plan["path_mu_stats"].get("mean", float("nan"))),
        "mu_safe": float(plan["feasibility_stats"].get("manip_mu_safe", float("nan"))),
        "reference_length_m": float(ref.get("reference_length_m", float("nan"))),
        "reference_length_ratio": float(ref.get("reference_length_ratio", float("nan"))),
        "c0_fallback_corner_count": int(ref.get("c0_fallback_corner_count", 0)),
        "accepted_blend_count": int(ref.get("accepted_blend_count", 0)),
        "success_basic": bool(control["success_basic"]),
        "success_strong": bool(control["success_strong"]),
        "final_pos_err_m": None if control["final_pos_err_m"] is None else float(control["final_pos_err_m"]),
        "final_rot_err_deg": None if control["final_rot_err_deg"] is None else float(control["final_rot_err_deg"]),
        "L_q": None if control["L_q"] is None else float(control["L_q"]),
        "L_q_j_max": None if control["L_q_j_max"] is None else float(control["L_q_j_max"]),
        "L_q_q2": _joint_value(0, "q2"),
        "L_q_q3": _joint_value(1, "q3"),
        "L_q_q5": _joint_value(2, "q5"),
        "rho_ref": None if control["rho_ref"] is None else float(control["rho_ref"]),
        "tracking_error_rms": None if control["tracking_error_rms"] is None else float(control["tracking_error_rms"]),
        "tracking_error_pos_rms": None if control["tracking_error_pos_rms"] is None else float(control["tracking_error_pos_rms"]),
        "peak_qdot": None if control["peak_qdot"] is None else float(control["peak_qdot"]),
        "lambda_active_ratio": None if control["lambda_active_ratio"] is None else float(control["lambda_active_ratio"]),
        "sigma_hard_breach_ratio": None if control["sigma_hard_breach_ratio"] is None else float(control["sigma_hard_breach_ratio"]),
    }


def _valid_pair(base: Dict[str, Any], cand: Dict[str, Any]) -> bool:
    if not (base["accepted"] and cand["accepted"]):
        return False
    keys = ("path_length_m", "L_q", "L_q_j_max", "L_q_q2", "L_q_q3", "L_q_q5", "rho_ref")
    return all(_finite_positive(base.get(k)) and _finite_positive(cand.get(k)) for k in keys)


def _ratio_row(split: str,
               scenario: ScenarioSpec,
               w: float,
               kappa: float,
               base: Dict[str, Any],
               cand: Dict[str, Any]) -> Dict[str, Any]:
    valid = _valid_pair(base, cand)
    row: Dict[str, Any] = {
        "split": split,
        "map_tag": base.get("map_tag"),
        "target_height": base.get("map_target_height"),
        "target_y": base.get("map_target_y"),
        "scenario": scenario.name,
        "obstacles": json.dumps([list(r) for r in scenario.obstacles]),
        "w": float(w),
        "kappa": float(kappa),
        "valid_pair": bool(valid),
        "base_accepted": bool(base["accepted"]),
        "cand_accepted": bool(cand["accepted"]),
        "base_reject": base["reject_reason"],
        "cand_reject": cand["reject_reason"],
        "base_L_q": base.get("L_q"),
        "cand_L_q": cand.get("L_q"),
        "base_L_q_j_max": base.get("L_q_j_max"),
        "cand_L_q_j_max": cand.get("L_q_j_max"),
        "base_L_q_q2": base.get("L_q_q2"),
        "cand_L_q_q2": cand.get("L_q_q2"),
        "base_L_q_q3": base.get("L_q_q3"),
        "cand_L_q_q3": cand.get("L_q_q3"),
        "base_L_q_q5": base.get("L_q_q5"),
        "cand_L_q_q5": cand.get("L_q_q5"),
        "base_rho_ref": base.get("rho_ref"),
        "cand_rho_ref": cand.get("rho_ref"),
        "base_path_length_m": base.get("path_length_m"),
        "cand_path_length_m": cand.get("path_length_m"),
        "base_nodes_expanded": base.get("nodes_expanded"),
        "cand_nodes_expanded": cand.get("nodes_expanded"),
        "base_planning_time_ms": base.get("planning_time_ms"),
        "cand_planning_time_ms": cand.get("planning_time_ms"),
        "cand_mu_safe": cand.get("mu_safe"),
        "cand_mu_p5_path": cand.get("mu_p5_path"),
    }
    if valid:
        J_q2 = cand["L_q_q2"] / base["L_q_q2"]
        J_q3 = cand["L_q_q3"] / base["L_q_q3"]
        J_q5 = cand["L_q_q5"] / base["L_q_q5"]
        J_q_values = (J_q2, J_q3, J_q5)
        row.update({
            "J_a": cand["L_q"] / base["L_q"],
            "J_b": cand["L_q_j_max"] / base["L_q_j_max"],
            "J_c": cand["rho_ref"] / base["rho_ref"],
            "J_q2": J_q2,
            "J_q3": J_q3,
            "J_q5": J_q5,
            "J_q_mean": float(np.mean(J_q_values)),
            "J_q_max": float(np.max(J_q_values)),
            "path_length_ratio": cand["path_length_m"] / base["path_length_m"],
            "nodes_ratio": cand["nodes_expanded"] / max(base["nodes_expanded"], 1),
            "planning_time_ratio": cand["planning_time_ms"] / max(base["planning_time_ms"], 1.0e-9),
        })
    else:
        row.update({
            "J_a": float("nan"),
            "J_b": float("nan"),
            "J_c": float("nan"),
            "J_q2": float("nan"),
            "J_q3": float("nan"),
            "J_q5": float("nan"),
            "J_q_mean": float("nan"),
            "J_q_max": float("nan"),
            "path_length_ratio": float("nan"),
            "nodes_ratio": float("nan"),
            "planning_time_ratio": float("nan"),
        })
    return row


def _mean(rows: Sequence[Dict[str, Any]], key: str) -> float:
    vals = [float(r[key]) for r in rows if math.isfinite(float(r.get(key, float("nan"))))]
    return float(np.mean(vals)) if vals else float("nan")


def _summarize_candidate(w: float, kappa: float, rows: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    supported_rows = [r for r in rows if bool(r["base_accepted"])]
    valid_rows = [r for r in rows if bool(r["valid_pair"])]
    n_total = len(rows)
    n_supported = len(supported_rows)
    n_valid = len(valid_rows)
    return {
        "w": float(w),
        "kappa": float(kappa),
        "n_total": int(n_total),
        "n_supported": int(n_supported),
        "n_valid": int(n_valid),
        "baseline_support_rate": float(n_supported / max(n_total, 1)),
        "valid_pair_rate": float(n_valid / max(n_supported, 1)),
        "mean_J_a": _mean(valid_rows, "J_a"),
        "mean_J_b": _mean(valid_rows, "J_b"),
        "mean_J_c": _mean(valid_rows, "J_c"),
        "mean_J_q2": _mean(valid_rows, "J_q2"),
        "mean_J_q3": _mean(valid_rows, "J_q3"),
        "mean_J_q5": _mean(valid_rows, "J_q5"),
        "mean_J_q_mean": _mean(valid_rows, "J_q_mean"),
        "mean_J_q_max": _mean(valid_rows, "J_q_max"),
        "mean_path_length_ratio": _mean(valid_rows, "path_length_ratio"),
        "max_path_length_ratio": float(np.max([float(r["path_length_ratio"]) for r in valid_rows])) if valid_rows else float("nan"),
        "mean_nodes_expanded": _mean(valid_rows, "cand_nodes_expanded"),
        "mean_planning_time_ms": _mean(valid_rows, "cand_planning_time_ms"),
        "win_Ja_count": int(sum(1 for r in valid_rows if float(r["J_a"]) < 1.0)),
        "win_Jc_count": int(sum(1 for r in valid_rows if float(r["J_c"]) < 1.0)),
        "win_Jq2_count": int(sum(1 for r in valid_rows if float(r["J_q2"]) < 1.0)),
        "win_Jq3_count": int(sum(1 for r in valid_rows if float(r["J_q3"]) < 1.0)),
        "win_Jq5_count": int(sum(1 for r in valid_rows if float(r["J_q5"]) < 1.0)),
    }


def _write_csv(path: str, rows: Sequence[Dict[str, Any]]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not rows:
        with open(path, "w", encoding="utf-8", newline="") as f:
            f.write("")
        return
    keys: List[str] = []
    for row in rows:
        for k in row.keys():
            if k not in keys:
                keys.append(k)
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for row in rows:
            w.writerow(row)


def _write_json(path: str, data: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _plot_heatmap(path: str,
                  rows: Sequence[Dict[str, Any]],
                  w_grid: Sequence[float],
                  k_grid: Sequence[float],
                  key: str,
                  title: str) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    data = np.full((len(k_grid), len(w_grid)), np.nan, dtype=np.float64)
    idx_w = {float(v): i for i, v in enumerate(w_grid)}
    idx_k = {float(v): i for i, v in enumerate(k_grid)}
    for row in rows:
        if key not in row:
            continue
        i = idx_k.get(float(row["kappa"]))
        j = idx_w.get(float(row["w"]))
        if i is None or j is None:
            continue
        try:
            data[i, j] = float(row[key])
        except Exception:
            pass
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig, ax = plt.subplots(figsize=(max(5.0, 0.55 * len(w_grid)), max(3.8, 0.45 * len(k_grid))), dpi=140)
    im = ax.imshow(data, origin="lower", aspect="auto", cmap="viridis")
    ax.set_xticks(range(len(w_grid)))
    ax.set_yticks(range(len(k_grid)))
    ax.set_xticklabels([f"{v:g}" for v in w_grid])
    ax.set_yticklabels([f"{v:g}" for v in k_grid])
    ax.set_xlabel("w")
    ax.set_ylabel("kappa")
    ax.set_title(title)
    fig.colorbar(im, ax=ax)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _select(summary_rows: Sequence[Dict[str, Any]],
            min_valid_rate: float,
            max_path_ratio: float) -> Tuple[Optional[Dict[str, Any]], Dict[str, Any]]:
    feasible: List[Dict[str, Any]] = []
    for row in summary_rows:
        if float(row["valid_pair_rate"]) < min_valid_rate:
            continue
        if not math.isfinite(float(row["max_path_length_ratio"])):
            continue
        if float(row["max_path_length_ratio"]) > max_path_ratio:
            continue
        if not (float(row["mean_J_a"]) < 1.0 or float(row["mean_J_c"]) < 1.0):
            continue
        feasible.append(row)
    report = {
        "min_valid_pair_rate": float(min_valid_rate),
        "max_path_length_ratio": float(max_path_ratio),
        "n_candidates": int(len(summary_rows)),
        "n_feasible_candidates": int(len(feasible)),
        "valid_pair_rate_denominator": "baseline-accepted comparable pairs",
        "selection_rule": [
            "valid_pair_rate desc",
            "max_path_length_ratio <= max_path_length_ratio_limit",
            "mean_J_a asc",
            "mean_J_c asc",
            "mean_J_q_max asc",
            "mean_J_b asc",
            "mean_J_q_mean asc",
            "w asc",
            "mean_nodes_expanded asc",
            "mean_planning_time_ms asc",
            "kappa asc",
        ],
    }
    if not feasible:
        report["selected"] = None
        report["reason"] = "No candidate improved mean J_a or mean J_c under the path-length and valid-pair constraints."
        return None, report
    feasible.sort(key=lambda r: (
        -float(r["valid_pair_rate"]),
        float(r["mean_J_a"]),
        float(r["mean_J_c"]),
        float(r["mean_J_q_max"]),
        float(r["mean_J_b"]),
        float(r["mean_J_q_mean"]),
        float(r["w"]),
        float(r["mean_nodes_expanded"]) if math.isfinite(float(r["mean_nodes_expanded"])) else float("inf"),
        float(r["mean_planning_time_ms"]) if math.isfinite(float(r["mean_planning_time_ms"])) else float("inf"),
        float(r["kappa"]),
    ))
    selected = feasible[0]
    report["selected"] = selected
    report["reason"] = "Selected best feasible candidate by lexicographic execution-quality rule."
    return selected, report


def _rank_candidate_rows(rows: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    ranked = list(rows)
    ranked.sort(key=lambda r: (
        -float(r["valid_pair_rate"]),
        float(r["mean_J_a"]) if math.isfinite(float(r["mean_J_a"])) else float("inf"),
        float(r["mean_J_c"]) if math.isfinite(float(r["mean_J_c"])) else float("inf"),
        float(r["mean_J_q_max"]) if math.isfinite(float(r["mean_J_q_max"])) else float("inf"),
        float(r["mean_J_b"]) if math.isfinite(float(r["mean_J_b"])) else float("inf"),
        float(r["mean_J_q_mean"]) if math.isfinite(float(r["mean_J_q_mean"])) else float("inf"),
        float(r["w"]),
        float(r["mean_nodes_expanded"]) if math.isfinite(float(r["mean_nodes_expanded"])) else float("inf"),
        float(r["mean_planning_time_ms"]) if math.isfinite(float(r["mean_planning_time_ms"])) else float("inf"),
        float(r["kappa"]),
    ))
    return ranked


def _best_grid_candidate(summary_rows: Sequence[Dict[str, Any]],
                         min_valid_rate: float,
                         max_path_ratio: float) -> Optional[Dict[str, Any]]:
    feasible = [
        r for r in summary_rows
        if float(r["valid_pair_rate"]) >= float(min_valid_rate)
        and math.isfinite(float(r["max_path_length_ratio"]))
        and float(r["max_path_length_ratio"]) <= float(max_path_ratio)
    ]
    if feasible:
        return _rank_candidate_rows(feasible)[0]
    valid = [r for r in summary_rows if float(r["valid_pair_rate"]) > 0.0]
    if valid:
        return _rank_candidate_rows(valid)[0]
    ranked = _rank_candidate_rows(summary_rows)
    return ranked[0] if ranked else None


def _candidate_summaries(rows: Sequence[Dict[str, Any]],
                         w_grid: Sequence[float],
                         k_grid: Sequence[float]) -> List[Dict[str, Any]]:
    return [
        _summarize_candidate(
            w_val,
            k_val,
            [
                r for r in rows
                if float(r["w"]) == float(w_val) and float(r["kappa"]) == float(k_val)
            ],
        )
        for w_val in w_grid
        for k_val in k_grid
    ]


def _height_key(row: Dict[str, Any]) -> float:
    return round(float(row["target_height"]), 6)


def _height_parameter_table(ratio_rows: Sequence[Dict[str, Any]],
                            w_grid: Sequence[float],
                            k_grid: Sequence[float],
                            args: argparse.Namespace) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    heights = sorted({_height_key(r) for r in ratio_rows})
    table: List[Dict[str, Any]] = []
    reports: Dict[str, Any] = {}
    for h in heights:
        rows_h = [r for r in ratio_rows if _height_key(r) == h]
        summaries_h = _candidate_summaries(rows_h, w_grid, k_grid)
        selected_h, report_h = _select(summaries_h, args.min_valid_pair_rate, args.max_path_ratio)
        best_h = selected_h if selected_h is not None else _best_grid_candidate(
            summaries_h,
            args.min_valid_pair_rate,
            args.max_path_ratio,
        )
        reports[f"{h:.6f}"] = report_h
        if best_h is None:
            table.append({
                "target_height": float(h),
                "selected": False,
                "passes_improvement_gate": False,
                "w": None,
                "kappa": None,
                "source": "no_valid_grid_candidate",
                "reason": "No valid grid candidate could be ranked.",
            })
        else:
            row = {
                "target_height": float(h),
                "selected": True,
                "passes_improvement_gate": bool(selected_h is not None),
                "source": "improving_candidate" if selected_h is not None else "best_grid_no_improvement",
            }
            row.update(best_h)
            if selected_h is None:
                row["reason"] = report_h.get("reason", "No candidate passed the improvement gate; best grid pair recorded for table completeness.")
            table.append(row)
    return table, reports


def _safe_corr(x: np.ndarray, y: np.ndarray) -> float:
    if x.size < 2 or y.size < 2:
        return float("nan")
    if float(np.std(x)) <= 1.0e-12 or float(np.std(y)) <= 1.0e-12:
        return float("nan")
    return float(np.corrcoef(x, y)[0, 1])


def _polyfit_report(h: np.ndarray, y: np.ndarray, degree: int) -> Dict[str, Any]:
    mask = np.isfinite(h) & np.isfinite(y)
    h = h[mask]
    y = y[mask]
    if h.size <= degree:
        return {"available": False, "degree": int(degree), "reason": "not enough selected heights"}
    if float(np.std(y)) <= 1.0e-12:
        return {
            "available": True,
            "degree": int(degree),
            "coefficients_descending": [0.0] * degree + [float(y[0])],
            "r2": float("nan"),
            "rmse": 0.0,
            "reason": "constant selected value",
        }
    try:
        coeff = np.polyfit(h, y, degree)
    except Exception as exc:
        return {"available": False, "degree": int(degree), "reason": str(exc)}
    pred = np.polyval(coeff, h)
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - float(np.mean(y))) ** 2))
    r2 = float(1.0 - ss_res / ss_tot) if ss_tot > 1.0e-18 else float("nan")
    rmse = float(np.sqrt(np.mean((y - pred) ** 2)))
    return {
        "available": True,
        "degree": int(degree),
        "coefficients_descending": [float(v) for v in coeff],
        "r2": r2,
        "rmse": rmse,
    }


def _mode_float(values: Sequence[float]) -> Optional[float]:
    if not values:
        return None
    counts: Dict[float, int] = {}
    for v in values:
        counts[float(v)] = counts.get(float(v), 0) + 1
    return sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))[0][0]


def _height_fit_report(table: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    selected_rows = [r for r in table if bool(r.get("selected")) and r.get("w") is not None and r.get("kappa") is not None]
    h = np.asarray([float(r["target_height"]) for r in selected_rows], dtype=np.float64)
    w = np.asarray([float(r["w"]) for r in selected_rows], dtype=np.float64)
    k = np.asarray([float(r["kappa"]) for r in selected_rows], dtype=np.float64)
    report: Dict[str, Any] = {
        "selected_height_count": int(len(selected_rows)),
        "w_unique": sorted({float(v) for v in w.tolist()}),
        "kappa_unique": sorted({float(v) for v in k.tolist()}),
        "w_mode": _mode_float(w.tolist()),
        "kappa_mode": _mode_float(k.tolist()),
        "pearson_corr_height_w": _safe_corr(h, w),
        "pearson_corr_height_kappa": _safe_corr(h, k),
        "w_linear": _polyfit_report(h, w, 1),
        "w_quadratic": _polyfit_report(h, w, 2),
        "kappa_linear": _polyfit_report(h, k, 1),
        "kappa_quadratic": _polyfit_report(h, k, 2),
    }
    if len(report["w_unique"]) <= 2 and len(report["kappa_unique"]) <= 2:
        report["interpretation_hint"] = (
            "Selected values are mostly discrete/plateau-like; a lookup table or mode value is "
            "more honest than a continuous formula unless future denser grids show smooth trends."
        )
    else:
        report["interpretation_hint"] = (
            "Check R2/RMSE before replacing the height table with a formula; polynomial fits are "
            "diagnostic summaries, not automatically selected runtime policies."
        )
    return report


def _request(scenario: ScenarioSpec,
             map_path: Optional[str],
             cost_mode: str,
             w: float,
             kappa: float,
             args: argparse.Namespace) -> MissionRequest:
    return MissionRequest(
        scenario=scenario.scenario,
        map_path=map_path,
        obstacles=scenario.obstacles,
        obstacle_inflation_m=float(args.inflation_m),
        map_update_hz=DEFAULT_MAP_UPDATE_HZ,
        mu_min=float(args.mu_min),
        cost_mode=cost_mode,
        manip_weight=float(w),
        manip_kappa=float(kappa),
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
        write_artifacts=bool(getattr(args, "save_cases", False)),
    )


def _request_from_job(job: Dict[str, Any]) -> MissionRequest:
    return MissionRequest(
        scenario=str(job["scenario"]),
        map_path=str(job["map_path"]),
        obstacles=tuple(tuple(r) for r in job["obstacles"]),
        obstacle_inflation_m=float(job["inflation_m"]),
        map_update_hz=DEFAULT_MAP_UPDATE_HZ,
        mu_min=float(job["mu_min"]),
        cost_mode=str(job["cost_mode"]),
        manip_weight=float(job["w"]),
        manip_kappa=float(job["kappa"]),
        manip_mu_safe=float(job["mu_safe"]),
        manip_mu_safe_percentile=float(job["mu_safe_percentile"]),
        reference_backend="c2_quintic",
        scheduler_mode="path_following",
        dt=float(job["dt"]),
        v_ref=float(job["v_ref"]),
        terminal_hold_s=float(job["terminal_hold_s"]),
        gap_target_m=float(job["gap_target_m"]),
        terminal_gap_eps_m=float(job["gap_target_m"]),
        make_plots=False,
        write_artifacts=bool(job.get("save_case", False)),
    )


def _run_eval_job(job: Dict[str, Any]) -> Dict[str, Any]:
    module = ControlModule(output_root=None)
    summary = module.run(_request_from_job(job), out_dir=job.get("out_dir"))
    metric = _extract(summary)
    return {
        "role": job["role"],
        "map_tag": job["map_tag"],
        "target_height": job["target_height"],
        "target_y": job["target_y"],
        "scenario_name": job["scenario_name"],
        "scenario": job["scenario"],
        "w": float(job["w"]),
        "kappa": float(job["kappa"]),
        "cost_mode": job["cost_mode"],
        "metric": metric,
    }


def _evaluate_jobs(jobs: Sequence[Dict[str, Any]], workers: int, label: str) -> List[Dict[str, Any]]:
    if not jobs:
        return []
    if workers == 1:
        out: List[Dict[str, Any]] = []
        for i, job in enumerate(jobs, 1):
            out.append(_run_eval_job(job))
            if i == 1 or i == len(jobs) or i % 100 == 0:
                print(f"[{label}] completed {i}/{len(jobs)}")
        return out
    max_workers = os.cpu_count() if workers <= 0 else workers
    max_workers = max(1, int(max_workers or 1))
    out = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=max_workers) as pool:
        futures = [pool.submit(_run_eval_job, job) for job in jobs]
        for i, fut in enumerate(concurrent.futures.as_completed(futures), 1):
            out.append(fut.result())
            if i == 1 or i == len(futures) or i % 100 == 0:
                print(f"[{label}] completed {i}/{len(futures)}")
    out.sort(key=lambda r: (
        str(r["map_tag"]),
        str(r["scenario_name"]),
        float(r["w"]),
        float(r["kappa"]),
        str(r["role"]),
    ))
    return out


def run(args: argparse.Namespace) -> int:
    w_grid = _parse_grid(args.w_grid)
    k_grid = _parse_grid(args.kappa_grid)
    map_specs = _discover_maps(args)
    root = os.path.abspath(args.out or os.path.join(
        os.getcwd(),
        "parameter_tuning",
        datetime.now().strftime("%Y%m%dT%H%M%S"),
    ))
    cases_root = os.path.join(root, "cases")
    os.makedirs(cases_root, exist_ok=True)

    baselines: Dict[str, Dict[str, Any]] = {}
    raw_rows: List[Dict[str, Any]] = []
    ratio_rows: List[Dict[str, Any]] = []
    scenarios = _scenario_set(args)
    print(f"[tuning] output={root}")
    print(
        f"[tuning] maps={len(map_specs)} scenarios={len(scenarios)} "
        f"candidates={len(w_grid) * len(k_grid)} workers={args.workers}"
    )
    print(f"[tuning] scenario_set={','.join(s.name for s in scenarios)}")
    print(
        f"[tuning] height_range=[{min(m.height for m in map_specs):.3f}, "
        f"{max(m.height for m in map_specs):.3f}] save_cases={bool(args.save_cases)}"
    )

    def make_job(role: str,
                 map_spec: MapSpec,
                 scenario: ScenarioSpec,
                 cost_mode: str,
                 w_val: float,
                 k_val: float) -> Dict[str, Any]:
        out_dir = None
        if args.save_cases:
            stem = "baseline" if role == "baseline" else f"w_{w_val:g}_k_{k_val:g}"
            out_dir = os.path.join(cases_root, stem, map_spec.tag, scenario.name)
        return {
            "role": role,
            "map_path": map_spec.path,
            "map_tag": map_spec.tag,
            "target_height": map_spec.height,
            "target_y": map_spec.target_y,
            "scenario_name": scenario.name,
            "scenario": scenario.scenario,
            "obstacles": scenario.obstacles,
            "cost_mode": cost_mode,
            "w": float(w_val),
            "kappa": float(k_val),
            "inflation_m": float(args.inflation_m),
            "mu_min": float(args.mu_min),
            "mu_safe": float(args.mu_safe),
            "mu_safe_percentile": float(args.mu_safe_percentile),
            "dt": float(args.dt),
            "v_ref": float(args.v_ref),
            "terminal_hold_s": float(args.terminal_hold_s),
            "gap_target_m": float(args.gap_target_m),
            "save_case": bool(args.save_cases),
            "out_dir": out_dir,
        }

    baseline_jobs = [
        make_job("baseline", map_spec, scenario, "distance", 0.0, 1.0)
        for map_spec in map_specs
        for scenario in scenarios
    ]
    baseline_results = _evaluate_jobs(baseline_jobs, int(args.workers), "baseline")
    for result in baseline_results:
        key = (str(result["map_tag"]), str(result["scenario_name"]))
        baselines[key] = result["metric"]
        raw_rows.append({
            "role": "baseline",
            "map_tag": result["map_tag"],
            "target_height": result["target_height"],
            "target_y": result["target_y"],
            "scenario": result["scenario_name"],
            "w": 0.0,
            "kappa": 1.0,
            **result["metric"],
        })
    print(f"[baseline] accepted={sum(1 for r in baseline_results if r['metric']['accepted'])}/{len(baseline_results)}")

    candidate_jobs = [
        make_job("candidate", map_spec, scenario, "manipulability", w_val, k_val)
        for w_val in w_grid
        for k_val in k_grid
        for map_spec in map_specs
        for scenario in scenarios
    ]
    candidate_results = _evaluate_jobs(candidate_jobs, int(args.workers), "candidate")
    scenario_lookup = {s.name: s for s in scenarios}
    for result in candidate_results:
        cand = result["metric"]
        scenario_name = str(result["scenario_name"])
        key = (str(result["map_tag"]), scenario_name)
        base = baselines[key]
        w_val = float(result["w"])
        k_val = float(result["kappa"])
        raw_rows.append({
            "role": "candidate",
            "map_tag": result["map_tag"],
            "target_height": result["target_height"],
            "target_y": result["target_y"],
            "scenario": scenario_name,
            "w": w_val,
            "kappa": k_val,
            **cand,
        })
        rr = _ratio_row("tune", scenario_lookup[scenario_name], w_val, k_val, base, cand)
        ratio_rows.append(rr)

    summary_rows = _candidate_summaries(ratio_rows, w_grid, k_grid)
    for cs in summary_rows:
        print(
            f"[cand w={cs['w']:g} k={cs['kappa']:g}] valid={cs['valid_pair_rate']:.2f} "
            f"Ja={cs['mean_J_a']:.4f} Jc={cs['mean_J_c']:.4f} "
            f"Jqmax={cs['mean_J_q_max']:.4f} meanLen={cs['mean_path_length_ratio']:.4f} "
            f"maxLen={cs['max_path_length_ratio']:.4f}"
        )
    selected, report = _select(summary_rows, args.min_valid_pair_rate, args.max_path_ratio)
    height_table, height_reports = _height_parameter_table(ratio_rows, w_grid, k_grid, args)
    height_fit = _height_fit_report(height_table)
    selected_config = {
        "selected": selected is not None,
        "cost_mode": "manipulability" if selected is not None else None,
        "w": None if selected is None else float(selected["w"]),
        "kappa": None if selected is None else float(selected["kappa"]),
        "all_heights": bool(args.all_heights),
        "map_count": int(len(map_specs)),
        "height_min": float(min(m.height for m in map_specs)),
        "height_max": float(max(m.height for m in map_specs)),
        "scenario_set": [s.name for s in scenarios],
        "mu_min": float(args.mu_min),
        "inflation_m": float(args.inflation_m),
        "mu_safe": float(args.mu_safe),
        "mu_safe_percentile": float(args.mu_safe_percentile),
        "reference_backend": "c2_quintic",
    }
    if selected is not None:
        selected_config["selected_summary"] = selected

    _write_csv(os.path.join(root, "raw_metrics.csv"), raw_rows)
    _write_csv(os.path.join(root, "scenario_ratios.csv"), ratio_rows)
    _write_csv(os.path.join(root, "candidate_summary.csv"), summary_rows)
    _write_csv(os.path.join(root, "height_parameter_table.csv"), height_table)
    _write_json(os.path.join(root, "height_parameter_table.json"), {
        "scenario_set": [s.name for s in scenarios],
        "selection_policy": "same lexicographic rule as global selection, applied independently per target height",
        "table": height_table,
        "height_reports": height_reports,
    })
    _write_json(os.path.join(root, "height_parameter_fit.json"), height_fit)
    _write_json(os.path.join(root, "selected_config.json"), selected_config)
    _write_json(os.path.join(root, "selection_report.json"), report)
    img_root = os.path.join(root, "img")
    if len(w_grid) > 1 or len(k_grid) > 1:
        _plot_heatmap(os.path.join(img_root, "mean_J_a.png"), summary_rows, w_grid, k_grid, "mean_J_a", "mean J_a")
        _plot_heatmap(os.path.join(img_root, "mean_J_c.png"), summary_rows, w_grid, k_grid, "mean_J_c", "mean J_c")
        _plot_heatmap(os.path.join(img_root, "mean_J_q2.png"), summary_rows, w_grid, k_grid, "mean_J_q2", "mean J_q2")
        _plot_heatmap(os.path.join(img_root, "mean_J_q3.png"), summary_rows, w_grid, k_grid, "mean_J_q3", "mean J_q3")
        _plot_heatmap(os.path.join(img_root, "mean_J_q5.png"), summary_rows, w_grid, k_grid, "mean_J_q5", "mean J_q5")
        _plot_heatmap(os.path.join(img_root, "mean_J_q_max.png"), summary_rows, w_grid, k_grid, "mean_J_q_max", "mean joint-wise max ratio")
        _plot_heatmap(os.path.join(img_root, "valid_pair_rate.png"), summary_rows, w_grid, k_grid, "valid_pair_rate", "valid pair rate")
        _plot_heatmap(os.path.join(img_root, "mean_path_length_ratio.png"), summary_rows, w_grid, k_grid, "mean_path_length_ratio", "mean path length ratio")
        _plot_heatmap(os.path.join(img_root, "max_path_length_ratio.png"), summary_rows, w_grid, k_grid, "max_path_length_ratio", "max path length ratio")

    print(
        f"[height-table] selected={sum(1 for r in height_table if r.get('selected'))}/{len(height_table)} "
        f"w_unique={height_fit['w_unique']} kappa_unique={height_fit['kappa_unique']}"
    )

    if selected is None:
        print("[tuning] selected=None")
        print(f"[tuning] reason={report['reason']}")
    else:
        print(
            f"[tuning] selected w={selected['w']:.6g} kappa={selected['kappa']:.6g} "
            f"valid={selected['valid_pair_rate']:.2f} Ja={selected['mean_J_a']:.4f} "
            f"Jb={selected['mean_J_b']:.4f} Jc={selected['mean_J_c']:.4f} "
            f"Jqmax={selected['mean_J_q_max']:.4f} maxLen={selected['max_path_length_ratio']:.4f}"
        )
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Tune reduced-active manipulability edge cost.")
    p.add_argument("--map-path", default=None)
    p.add_argument("--all-heights", action="store_true",
                   help="Tune over all 0.047y height maps in the selected map directory.")
    p.add_argument("--map-dir", default=None)
    p.add_argument("--map-prefix", default="map_0.047y_")
    p.add_argument("--height-min", type=float, default=None)
    p.add_argument("--height-max", type=float, default=None)
    p.add_argument("--height-limit", type=int, default=0,
                   help="Debug limit for the number of selected height maps. 0 means no limit.")
    p.add_argument("--out", default=None)
    p.add_argument("--workers", type=int, default=0,
                   help="Parallel worker count. 0 uses all CPU cores; 1 runs serially.")
    p.add_argument("--save-cases", action="store_true",
                   help="Write per-case path/reference/episode artifacts. Off by default for large sweeps.")
    p.add_argument("--include-obstacles", action="store_true",
                   help="Include obstacle scenarios. Default is clear-only height calibration.")
    p.add_argument("--w-grid", default=DEFAULT_W_GRID,
                   help="Comma-separated w grid. Default: 0.05:0.05:1.50.")
    p.add_argument("--kappa-grid", default=DEFAULT_KAPPA_GRID,
                   help="Comma-separated kappa grid. Default: 0.5:0.5:10.0.")
    p.add_argument("--mu-min", type=float, default=DEFAULT_MU_MIN)
    p.add_argument("--inflation-m", type=float, default=DEFAULT_OBSTACLE_INFLATION_M)
    p.add_argument("--mu-safe", type=float, default=DEFAULT_MANIP_MU_SAFE)
    p.add_argument("--mu-safe-percentile", type=float, default=DEFAULT_MANIP_MU_SAFE_PERCENTILE)
    p.add_argument("--max-path-ratio", type=float, default=1.05)
    p.add_argument("--min-valid-pair-rate", type=float, default=1.0)
    p.add_argument("--dt", type=float, default=0.01)
    p.add_argument("--v-ref", type=float, default=0.04)
    p.add_argument("--terminal-hold-s", type=float, default=1.0)
    p.add_argument("--gap-target-m", type=float, default=0.01)
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
