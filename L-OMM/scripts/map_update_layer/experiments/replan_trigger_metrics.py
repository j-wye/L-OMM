#!/usr/bin/env python3
"""Aggregate synthetic replanning-trigger precision/recall metrics.

The script consumes dynamic_replanning_summary.json artifacts written by
run_dynamic_replanning_checks.py.  It does not use a camera, ROS2 topic, or
robot command; it only evaluates already-generated synthetic/stored artifacts.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from collections import defaultdict
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, "..", ".."))

if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(0, CONTROL_MODULE_DIR)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(1, SCRIPTS_DIR)

from map_update_layer.path_collision_monitor import PathCollisionMonitor  # noqa: E402


POSITIVE_EVENTS = {
    "CURRENT_POSE_UNSAFE",
    "TARGET_CHANGED",
    "GOAL_CORRIDOR_BLOCKED",
    "REFERENCE_BLOCKED",
}
NEGATIVE_EVENTS = {"MASK_CHANGED_NONCRITICAL", "NO_RELEVANT_CHANGE"}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Build replanning-trigger confusion matrices from synthetic dynamic "
            "map-update artifacts. No live camera, ROS2, or hardware execution."
        )
    )
    parser.add_argument(
        "--input",
        action="append",
        default=[],
        help="dynamic_replanning_summary.json file or a directory containing such files. Can be repeated.",
    )
    parser.add_argument("--out", default=None, help="Output directory for CSV/JSON/PNG metrics.")
    parser.add_argument("--min-precision", type=float, default=0.95)
    parser.add_argument("--min-recall", type=float, default=0.98)
    parser.add_argument("--min-safety-recall", type=float, default=1.0)
    parser.add_argument("--max-negative-false-trigger-rate", type=float, default=0.02)
    parser.add_argument(
        "--derive-missing-ground-truth",
        action="store_true",
        help="Fallback to expected_event parsing when frame metadata is absent. Prefer rerunning with explicit metadata.",
    )
    parser.add_argument(
        "--synthetic-path-collision-monitor",
        action="store_true",
        help="Generate a deterministic synthetic PathCollisionMonitor trigger trace instead of reading artifacts.",
    )
    parser.add_argument("--synthetic-frames", type=int, default=24)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "replan_trigger_metrics"))
    os.makedirs(out_root, exist_ok=True)
    if bool(args.synthetic_path_collision_monitor):
        summaries = []
        rows = _synthetic_path_collision_monitor_rows(frames=int(args.synthetic_frames))
    else:
        if not args.input:
            raise SystemExit("--input is required unless --synthetic-path-collision-monitor is set")
        summaries = _discover_summaries(args.input)
        rows = _collect_rows(summaries, derive_missing=bool(args.derive_missing_ground_truth))
    metrics = _compute_metrics(rows)
    path_collision_extension = _path_collision_extension_summary(rows)
    summary = {
        "mode": "replan_trigger_precision_recall",
        "explicitly_no_live_camera": True,
        "explicitly_no_ros2_topic": True,
        "explicitly_no_physical_robot_execution": True,
        "input_summary_count": len(summaries),
        "event_count_total": len(rows),
        "event_count_used": int(sum(1 for row in rows if row["used_in_metrics"])),
        "event_count_excluded": int(sum(1 for row in rows if not row["used_in_metrics"])),
        "thresholds": {
            "min_precision": float(args.min_precision),
            "min_recall": float(args.min_recall),
            "min_safety_recall": float(args.min_safety_recall),
            "max_negative_false_trigger_rate": float(args.max_negative_false_trigger_rate),
        },
        "metrics": metrics,
        "path_collision_monitor_extension": path_collision_extension,
        "pass": bool(
            metrics["overall"]["precision"] >= float(args.min_precision)
            and metrics["overall"]["recall"] >= float(args.min_recall)
            and metrics["safety_critical"]["recall"] >= float(args.min_safety_recall)
            and metrics["negative_class"]["false_trigger_rate"] <= float(args.max_negative_false_trigger_rate)
        ),
    }
    _write_event_rows(os.path.join(out_root, "replan_trigger_events.csv"), rows)
    _write_event_rows(os.path.join(out_root, "path_collision_monitor_trigger_evidence.csv"), rows)
    _write_confusion_matrix(os.path.join(out_root, "replan_trigger_confusion_matrix.csv"), metrics)
    _write_class_metrics(os.path.join(out_root, "replan_trigger_event_class_metrics.csv"), metrics)
    _write_json(os.path.join(out_root, "replan_trigger_metrics_summary.json"), summary)
    _write_confusion_png(os.path.join(out_root, "replan_trigger_confusion_matrix.png"), metrics)
    print(
        "[replan_trigger_metrics] "
        f"used={summary['event_count_used']} "
        f"precision={metrics['overall']['precision']:.3f} "
        f"recall={metrics['overall']['recall']:.3f} "
        f"pass={summary['pass']} "
        f"out={out_root}"
    )
    return 0 if summary["pass"] else 1


# === DIAGNOSTIC LOGGING (REMOVABLE) - JETSON_SENSITIVITY_SWEEP ===
class _SyntheticCapsule:
    def q_hits_mask(self, q_active: np.ndarray, handle: object, mask: np.ndarray, dilated_masks: object = None) -> tuple[bool, int, int]:
        q = np.asarray(q_active, dtype=np.float64).reshape(-1)
        hit = bool(q[0] >= 0.5)
        return hit, int(hit), 1

    def build_dilated_masks(self, handle: object, mask: np.ndarray) -> None:
        return None


class _SyntheticHandle:
    resolution_m = 0.01
    x0 = 0.0
    z0 = 0.0


def _synthetic_path_collision_monitor_rows(*, frames: int) -> List[Dict[str, Any]]:
    monitor = PathCollisionMonitor(_SyntheticCapsule(), handle=_SyntheticHandle())
    blocked = np.zeros((8, 8), dtype=bool)
    obstacle_inserted_frame = max(3, int(frames) // 3)
    path_overlap_frame = obstacle_inserted_frame + 2
    rows: List[Dict[str, Any]] = []
    for frame in range(int(frames)):
        q = np.zeros((8, 3), dtype=np.float64)
        if frame >= path_overlap_frame:
            q[3, 0] = 0.8
        report = monitor.check({"q_traj": q}, blocked)
        truth_positive = frame >= path_overlap_frame
        actual_event = "REFERENCE_BLOCKED" if report.is_collision else "NO_RELEVANT_CHANGE"
        truth_event = "REFERENCE_BLOCKED" if truth_positive else "NO_REPLAN_REQUIRED"
        rows.append(
            {
                "summary_path": "synthetic_path_collision_monitor",
                "cost_mode": "synthetic",
                "scenario": "path_collision_monitor_trigger",
                "frame_index": int(frame),
                "frame_label": f"frame_{frame:03d}",
                "expected_event": truth_event,
                "actual_event_class": actual_event,
                "actual_positive": bool(report.is_collision),
                "ground_truth_replan_required": bool(truth_positive),
                "ground_truth_event_class": truth_event,
                "ground_truth_event_classes": truth_event,
                "ground_truth_source": "synthetic_path_collision_oracle",
                "used_in_metrics": True,
                "excluded_reason": "",
                "obstacle_inserted_frame": int(obstacle_inserted_frame),
                "path_overlap_frame": int(path_overlap_frame),
                "path_collision_is_collision": bool(report.is_collision),
                "path_collision_colliding_cells_count": int(report.colliding_cells_count),
                "path_collision_checked_sample_count": int(report.checked_sample_count),
            }
        )
    return rows


def _path_collision_extension_summary(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    used = [row for row in rows if row.get("used_in_metrics")]
    false_replan = int(sum(1 for row in used if (not bool(row["ground_truth_replan_required"])) and bool(row["actual_positive"])))
    missed_replan = int(sum(1 for row in used if bool(row["ground_truth_replan_required"]) and not bool(row["actual_positive"])))
    inserted = [
        int(row["obstacle_inserted_frame"])
        for row in used
        if row.get("obstacle_inserted_frame") is not None
    ]
    detections = [
        int(row["frame_index"])
        for row in used
        if bool(row.get("path_collision_is_collision", False))
    ]
    trigger_latency = None
    if inserted and detections:
        trigger_latency = int(min(detections) - min(inserted))
    return {
        "path_collision_monitor_evidence_rows": int(sum(1 for row in used if "path_collision_is_collision" in row)),
        "false_replan_count": int(false_replan),
        "missed_replan_count": int(missed_replan),
        "trigger_latency_frames": trigger_latency,
    }
# === END DIAGNOSTIC LOGGING ===


def _discover_summaries(inputs: Iterable[str]) -> List[Tuple[str, Dict[str, Any]]]:
    paths: List[str] = []
    for item in inputs:
        path = os.path.abspath(item)
        if os.path.isdir(path):
            for root, _, files in os.walk(path):
                if "dynamic_replanning_summary.json" in files:
                    paths.append(os.path.join(root, "dynamic_replanning_summary.json"))
        else:
            paths.append(path)
    unique = sorted(dict.fromkeys(paths))
    summaries: List[Tuple[str, Dict[str, Any]]] = []
    for path in unique:
        with open(path, "r", encoding="utf-8") as f:
            summaries.append((path, json.load(f)))
    return summaries


def _collect_rows(
    summaries: List[Tuple[str, Dict[str, Any]]],
    *,
    derive_missing: bool,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for summary_path, summary in summaries:
        mode = str(summary.get("cost_mode", _mode_from_path(summary_path)))
        for scenario_name, scenario in dict(summary.get("scenarios", {})).items():
            for event in list(scenario.get("events", [])):
                row = _event_row(
                    summary_path=summary_path,
                    cost_mode=mode,
                    scenario_name=str(scenario_name),
                    event=event,
                    derive_missing=derive_missing,
                )
                rows.append(row)
    return rows


def _event_row(
    *,
    summary_path: str,
    cost_mode: str,
    scenario_name: str,
    event: Dict[str, Any],
    derive_missing: bool,
) -> Dict[str, Any]:
    actual_event = str(event.get("event_class", ""))
    actual_positive = actual_event in POSITIVE_EVENTS
    gt_defined = isinstance(event.get("ground_truth_replan_required"), bool)
    gt_use = bool(event.get("ground_truth_use_in_metrics", gt_defined))
    gt_event_text = str(event.get("ground_truth_event_class", ""))
    if gt_defined:
        truth_positive = bool(event.get("ground_truth_replan_required"))
        truth_classes = _split_classes(gt_event_text)
        truth_source = "frame_metadata"
    elif derive_missing:
        truth_classes = _split_classes(str(event.get("expected_event", "")))
        truth_positive = bool(truth_classes & POSITIVE_EVENTS)
        truth_source = "expected_event_fallback"
    else:
        truth_classes = set()
        truth_positive = False
        truth_source = "missing"
        gt_use = False
    if not gt_event_text and truth_classes:
        gt_event_text = "|".join(sorted(truth_classes))
    used = bool(gt_use and (gt_defined or derive_missing))
    if truth_positive and not (truth_classes & POSITIVE_EVENTS):
        truth_classes = {"POSITIVE_TRIGGER"}
    if (not truth_positive) and not truth_classes:
        truth_classes = {"NO_REPLAN_REQUIRED"}
    return {
        "summary_path": summary_path,
        "cost_mode": cost_mode,
        "scenario": scenario_name,
        "frame_index": event.get("frame_index"),
        "frame_label": str(event.get("frame_label", "")),
        "expected_event": str(event.get("expected_event", "")),
        "actual_event_class": actual_event,
        "actual_positive": bool(actual_positive),
        "ground_truth_replan_required": bool(truth_positive),
        "ground_truth_event_class": gt_event_text,
        "ground_truth_event_classes": "|".join(sorted(truth_classes)),
        "ground_truth_source": truth_source,
        "used_in_metrics": bool(used),
        "excluded_reason": "" if used else str(event.get("ground_truth_oracle_status", "ground_truth_missing_or_disabled")),
    }


def _compute_metrics(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    used = [row for row in rows if row["used_in_metrics"]]
    tp = int(sum(1 for row in used if row["ground_truth_replan_required"] and row["actual_positive"]))
    fp = int(sum(1 for row in used if (not row["ground_truth_replan_required"]) and row["actual_positive"]))
    tn = int(sum(1 for row in used if (not row["ground_truth_replan_required"]) and (not row["actual_positive"])))
    fn = int(sum(1 for row in used if row["ground_truth_replan_required"] and (not row["actual_positive"])))
    overall = _binary_metrics(tp, fp, tn, fn)
    safety_rows = [row for row in used if row["ground_truth_replan_required"]]
    safety_tp = int(sum(1 for row in safety_rows if row["actual_positive"]))
    safety_fn = int(sum(1 for row in safety_rows if not row["actual_positive"]))
    negative_rows = [row for row in used if not row["ground_truth_replan_required"]]
    negative_fp = int(sum(1 for row in negative_rows if row["actual_positive"]))
    negative_tn = int(sum(1 for row in negative_rows if not row["actual_positive"]))
    class_metrics: Dict[str, Dict[str, Any]] = {}
    class_rows = [
        row for row in used
        if len(_split_classes(row["ground_truth_event_classes"])) == 1
    ]
    for cls in sorted(POSITIVE_EVENTS | {"MASK_CHANGED_NONCRITICAL", "NO_RELEVANT_CHANGE"}):
        cls_tp = int(sum(1 for row in class_rows if cls in _split_classes(row["ground_truth_event_classes"]) and row["actual_event_class"] == cls))
        cls_fp = int(sum(1 for row in class_rows if cls not in _split_classes(row["ground_truth_event_classes"]) and row["actual_event_class"] == cls))
        cls_fn = int(sum(1 for row in class_rows if cls in _split_classes(row["ground_truth_event_classes"]) and row["actual_event_class"] != cls))
        cls_tn = int(len(class_rows) - cls_tp - cls_fp - cls_fn)
        class_metrics[cls] = _binary_metrics(cls_tp, cls_fp, cls_tn, cls_fn)
        class_metrics[cls]["support"] = int(sum(1 for row in class_rows if cls in _split_classes(row["ground_truth_event_classes"])))
        class_metrics[cls]["ambiguous_class_rows_excluded"] = int(len(used) - len(class_rows))
    return {
        "overall": overall,
        "safety_critical": {
            "tp": safety_tp,
            "fn": safety_fn,
            "recall": _safe_div(safety_tp, safety_tp + safety_fn),
        },
        "negative_class": {
            "tn": negative_tn,
            "fp": negative_fp,
            "false_trigger_rate": _safe_div(negative_fp, negative_fp + negative_tn),
        },
        "event_class_metrics": class_metrics,
    }


def _binary_metrics(tp: int, fp: int, tn: int, fn: int) -> Dict[str, Any]:
    precision = _safe_div(tp, tp + fp)
    recall = _safe_div(tp, tp + fn)
    f1 = _safe_div(2.0 * precision * recall, precision + recall)
    return {
        "tp": int(tp),
        "fp": int(fp),
        "tn": int(tn),
        "fn": int(fn),
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(f1),
    }


def _safe_div(num: float, den: float) -> float:
    return float(num / den) if den else 0.0


def _split_classes(text: str) -> Set[str]:
    out: Set[str] = set()
    for chunk in str(text or "").split("|"):
        for token in chunk.split(","):
            cls = token.strip()
            if cls:
                out.add(cls)
    return out


def _mode_from_path(path: str) -> str:
    parts = os.path.normpath(path).split(os.sep)
    for part in reversed(parts):
        if part in {"distance", "manipulability"}:
            return part
    return ""


def _write_event_rows(path: str, rows: List[Dict[str, Any]]) -> None:
    keys = [
        "summary_path",
        "cost_mode",
        "scenario",
        "frame_index",
        "frame_label",
        "expected_event",
        "actual_event_class",
        "actual_positive",
        "ground_truth_replan_required",
        "ground_truth_event_class",
        "ground_truth_event_classes",
        "ground_truth_source",
        "used_in_metrics",
        "excluded_reason",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in keys})


def _write_confusion_matrix(path: str, metrics: Dict[str, Any]) -> None:
    overall = metrics["overall"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["truth", "predicted_replan", "predicted_no_replan"])
        writer.writeheader()
        writer.writerow({"truth": "replan_required", "predicted_replan": overall["tp"], "predicted_no_replan": overall["fn"]})
        writer.writerow({"truth": "no_replan_required", "predicted_replan": overall["fp"], "predicted_no_replan": overall["tn"]})


def _write_class_metrics(path: str, metrics: Dict[str, Any]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        keys = [
            "event_class",
            "tp",
            "fp",
            "tn",
            "fn",
            "precision",
            "recall",
            "f1",
            "support",
            "ambiguous_class_rows_excluded",
        ]
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for cls, row in metrics["event_class_metrics"].items():
            payload = {"event_class": cls}
            payload.update(row)
            writer.writerow(payload)


def _write_json(path: str, data: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _write_confusion_png(path: str, metrics: Dict[str, Any]) -> Optional[str]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception:
        return None
    overall = metrics["overall"]
    matrix = np.asarray([[overall["tp"], overall["fn"]], [overall["fp"], overall["tn"]]], dtype=float)
    labels = [["TP", "FN"], ["FP", "TN"]]
    fig, ax = plt.subplots(figsize=(4.8, 4.2), dpi=150)
    ax.imshow(matrix, cmap="Blues")
    ax.set_xticks([0, 1], ["pred replan", "pred no-replan"])
    ax.set_yticks([0, 1], ["truth replan", "truth no-replan"])
    for i in range(2):
        for j in range(2):
            ax.text(j, i, f"{labels[i][j]}\n{int(matrix[i, j])}", ha="center", va="center", color="black")
    ax.set_title("Replan trigger confusion matrix")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


if __name__ == "__main__":
    raise SystemExit(main())
