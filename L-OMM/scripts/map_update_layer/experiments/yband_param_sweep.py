#!/usr/bin/env python3
"""Synthetic tau_cover / delta_y sweep for task-plane y-band filtering."""
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

if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(0, CONTROL_MODULE_DIR)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(1, SCRIPTS_DIR)

from constants import Y_PLANE_FIXED  # noqa: E402
from map_update_layer.perception_to_map import _point3d_yband_filter  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run a synthetic tau_cover/delta_y trade-off sweep. "
            "No live camera, ROS2 topic, or physical robot execution."
        )
    )
    parser.add_argument("--out", default=None)
    parser.add_argument("--tau-cover", default="0.02,0.05,0.08,0.10")
    parser.add_argument("--delta-y", default="0.06,0.08,0.09,0.10,0.12")
    parser.add_argument("--k-sigma", type=float, default=2.5)
    parser.add_argument(
        "--depth-relevant-max-m",
        default="1.0",
        help="Comma-separated DEPTH_RELEVANT_MAX_M values to cross with tau_cover and delta_y.",
    )
    parser.add_argument("--samples-per-object", type=int, default=80)
    parser.add_argument("--seed", type=int, default=7)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "yband_param_sweep"))
    os.makedirs(out_root, exist_ok=True)
    tau_values = _float_list(args.tau_cover)
    delta_values = _float_list(args.delta_y)
    depth_caps = _float_list(args.depth_relevant_max_m)
    objects = _synthetic_objects(samples_per_object=int(args.samples_per_object), seed=int(args.seed))
    rows: List[Dict[str, Any]] = []
    for depth_cap in depth_caps:
        for tau in tau_values:
            for delta_y in delta_values:
                counts = {"tp": 0, "fp": 0, "tn": 0, "fn": 0}
                coverage_values: List[float] = []
                for obj in objects:
                    if float(np.mean(obj["depth"])) > float(depth_cap):
                        mask = np.zeros_like(obj["depth"], dtype=bool)
                        coverage = 0.0
                    else:
                        mask, coverage = _point3d_yband_filter(
                            obj["points_base"],
                            obj["depth"],
                            obj["u"],
                            y_plane=Y_PLANE_FIXED,
                            tau_cover=float(tau),
                            delta_y_static_m=float(delta_y),
                            k_sigma=float(args.k_sigma),
                        )
                    predicted = bool(np.any(mask))
                    truth = bool(obj["task_relevant"])
                    coverage_values.append(float(coverage))
                    if truth and predicted:
                        counts["tp"] += 1
                    elif truth and not predicted:
                        counts["fn"] += 1
                    elif (not truth) and predicted:
                        counts["fp"] += 1
                    else:
                        counts["tn"] += 1
                metric = _metrics(counts)
                rows.append(
                    {
                        "depth_relevant_max_m": float(depth_cap),
                        "tau_cover": float(tau),
                        "delta_y_m": float(delta_y),
                        "k_sigma": float(args.k_sigma),
                        "object_count": len(objects),
                        "coverage_mean": float(np.mean(coverage_values)),
                        "coverage_p05": float(np.percentile(coverage_values, 5.0)),
                        "coverage_p95": float(np.percentile(coverage_values, 95.0)),
                        **counts,
                        **metric,
                    }
                )
    best = max(rows, key=lambda r: (r["f1"], r["recall"], -r["false_positive_rate"]))
    summary = {
        "mode": "synthetic_yband_param_sweep",
        "explicitly_no_live_camera": True,
        "explicitly_no_ros2_topic": True,
        "explicitly_no_physical_robot_execution": True,
        "tau_cover_values": tau_values,
        "delta_y_values_m": delta_values,
        "depth_relevant_max_m_values": depth_caps,
        "synthetic_object_count": len(objects),
        "best_by_f1": best,
        "default_candidate": _find_default(rows),
        "pass": bool(rows),
        "acceptance_note": "P2.1 is a trade-off measurement; use best_by_f1/default_candidate to decide whether defaults should change.",
    }
    _write_csv(os.path.join(out_root, "yband_param_sweep.csv"), rows)
    _write_json(os.path.join(out_root, "yband_param_sweep_summary.json"), summary)
    _write_heatmaps(out_root, rows, tau_values, delta_values)
    print(
        "[yband_param_sweep] "
        f"objects={len(objects)} best_depth_cap={best['depth_relevant_max_m']} "
        f"best_tau={best['tau_cover']} best_delta_y={best['delta_y_m']} "
        f"f1={best['f1']:.3f} out={out_root}"
    )
    return 0 if summary["pass"] else 1


def _synthetic_objects(*, samples_per_object: int, seed: int) -> List[Dict[str, Any]]:
    rng = np.random.default_rng(seed)
    depths = np.asarray([0.3, 0.5, 0.8, 1.0, 1.5], dtype=np.float64)
    y_offsets = np.asarray([-0.16, -0.12, -0.09, -0.06, -0.03, 0.0, 0.03, 0.06, 0.09, 0.12, 0.16], dtype=np.float64)
    sizes = np.asarray([0.02, 0.04], dtype=np.float64)
    objects: List[Dict[str, Any]] = []
    for depth in depths:
        for y_offset in y_offsets:
            for size in sizes:
                jitter = rng.normal(loc=0.0, scale=max(float(size) * 0.20, 1.0e-6), size=(samples_per_object, 3))
                pts = np.zeros((samples_per_object, 3), dtype=np.float64)
                pts[:, 0] = 0.25 + jitter[:, 0]
                pts[:, 1] = Y_PLANE_FIXED + float(y_offset) + jitter[:, 1]
                pts[:, 2] = 0.50 + jitter[:, 2]
                depth_vec = np.full(samples_per_object, float(depth), dtype=np.float64)
                u = 319.5 + (pts[:, 1] - Y_PLANE_FIXED) * 615.0 / max(float(depth), 1.0e-6)
                task_relevant = abs(float(y_offset)) <= 0.091
                objects.append(
                    {
                        "points_base": pts,
                        "depth": depth_vec,
                        "u": u,
                        "task_relevant": bool(task_relevant),
                    }
                )
    return objects


def _metrics(counts: Dict[str, int]) -> Dict[str, float]:
    tp, fp, tn, fn = counts["tp"], counts["fp"], counts["tn"], counts["fn"]
    precision = _safe_div(tp, tp + fp)
    recall = _safe_div(tp, tp + fn)
    f1 = _safe_div(2.0 * precision * recall, precision + recall)
    fpr = _safe_div(fp, fp + tn)
    fnr = _safe_div(fn, fn + tp)
    return {
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "false_positive_rate": fpr,
        "false_negative_rate": fnr,
    }


def _safe_div(num: float, den: float) -> float:
    return float(num / den) if den else 0.0


def _float_list(text: str) -> List[float]:
    return [float(v.strip()) for v in str(text).split(",") if v.strip()]


def _find_default(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    candidates = [
        row for row in rows
        if abs(float(row["tau_cover"]) - 0.05) < 1.0e-12
        and abs(float(row["delta_y_m"]) - 0.09) < 1.0e-12
        and abs(float(row.get("depth_relevant_max_m", 1.0)) - 1.0) < 1.0e-12
    ]
    return candidates[0] if candidates else {}


def _write_csv(path: str, rows: List[Dict[str, Any]]) -> None:
    keys = list(rows[0].keys()) if rows else []
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def _write_json(path: str, data: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _write_heatmaps(out_root: str, rows: List[Dict[str, Any]], tau_values: List[float], delta_values: List[float]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    depth_caps = sorted({float(row.get("depth_relevant_max_m", 1.0)) for row in rows})
    for depth_cap in depth_caps:
        depth_rows = [row for row in rows if abs(float(row.get("depth_relevant_max_m", 1.0)) - depth_cap) < 1.0e-12]
        suffix = f"depth_cap_{str(depth_cap).replace('.', 'p')}"
        for metric_name in ("recall", "false_positive_rate"):
            grid = np.zeros((len(tau_values), len(delta_values)), dtype=float)
            for row in depth_rows:
                i = tau_values.index(float(row["tau_cover"]))
                j = delta_values.index(float(row["delta_y_m"]))
                grid[i, j] = float(row[metric_name])
            fig, ax = plt.subplots(figsize=(6.0, 4.0), dpi=150)
            im = ax.imshow(grid, origin="lower", cmap="viridis", vmin=0.0, vmax=1.0)
            ax.set_xticks(range(len(delta_values)), [f"{v:.2f}" for v in delta_values])
            ax.set_yticks(range(len(tau_values)), [f"{v:.2f}" for v in tau_values])
            ax.set_xlabel("delta_y [m]")
            ax.set_ylabel("tau_cover")
            ax.set_title(f"{metric_name}, depth_cap={depth_cap:.2f} m")
            fig.colorbar(im, ax=ax)
            fig.tight_layout()
            fig.savefig(os.path.join(out_root, f"yband_{metric_name}_heatmap_{suffix}.png"))
            plt.close(fig)
    fig, ax = plt.subplots(figsize=(5.0, 4.0), dpi=150)
    ax.scatter([row["recall"] for row in rows], [row["precision"] for row in rows], c=[row["delta_y_m"] for row in rows])
    ax.set_xlabel("recall")
    ax.set_ylabel("precision")
    ax.set_xlim(0.0, 1.01)
    ax.set_ylim(0.0, 1.01)
    ax.set_title("Y-band precision/recall")
    fig.tight_layout()
    fig.savefig(os.path.join(out_root, "yband_precision_recall_curve.png"))
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
