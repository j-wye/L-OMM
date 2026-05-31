#!/usr/bin/env python3
"""Synthetic PathCollisionMonitor metrics.

The script measures monitor-call latency and trigger quality using deterministic
synthetic references.  It never creates publishers, action clients, ROS2
subscribers, or robot commands.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

import numpy as np


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, "..", ".."))

if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(0, CONTROL_MODULE_DIR)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(1, SCRIPTS_DIR)

from map_update_layer.path_collision_monitor import PathCollisionMonitor  # noqa: E402


@dataclass(frozen=True)
class SyntheticCase:
    case_id: str
    frame_index: int
    q_traj: np.ndarray
    truth_collision: bool
    obstacle_inserted: bool


class SyntheticCapsule:
    def __init__(self, hit_threshold: float = 0.5) -> None:
        self.hit_threshold = float(hit_threshold)

    def q_hits_mask(self, q_active: np.ndarray, handle: Any, mask: np.ndarray, dilated_masks: Any = None) -> tuple[bool, int, int]:
        q = np.asarray(q_active, dtype=np.float64).reshape(-1)
        hit = bool(q[0] >= self.hit_threshold)
        return hit, int(hit), 1

    def build_dilated_masks(self, handle: Any, mask: np.ndarray) -> None:
        return None


class SyntheticHandle:
    resolution_m = 0.01
    x0 = 0.0
    z0 = 0.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Measure synthetic PathCollisionMonitor trigger metrics. "
            "No live camera, ROS2 topic, or physical robot execution."
        )
    )
    parser.add_argument("--out", default=None)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--frames", type=int, default=24)
    parser.add_argument("--iterations", type=int, default=100)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = Path(args.out or os.path.join(PROJECT_ROOT, "path", "path_collision_monitor_metrics"))
    out_root.mkdir(parents=True, exist_ok=True)
    cases = synthetic_cases(frames=int(args.frames), seed=int(args.seed))
    eval_rows, latency_samples = evaluate_cases(cases, iterations=int(args.iterations))
    latency = latency_distribution(latency_samples)
    detection_rows = detection_latency_rows(eval_rows)
    confusion = confusion_matrix(eval_rows)
    summary = {
        "mode": "path_collision_monitor_metrics",
        "explicitly_no_live_camera": True,
        "explicitly_no_ros2_topic": True,
        "explicitly_no_physical_robot_execution": True,
        "frame_count": int(args.frames),
        "case_count": int(len(cases)),
        "latency": latency,
        "confusion_matrix": confusion,
        "detection_latency_frames_min": min((r["detection_latency_frames"] for r in detection_rows), default=None),
        "detection_latency_frames_max": max((r["detection_latency_frames"] for r in detection_rows), default=None),
        "pass": bool(confusion["tp"] >= 1 and confusion["fp"] >= 1 and confusion["fn"] >= 1 and latency["sample_count"] > 0),
    }
    write_json(out_root / "check_latency_distribution.json", latency)
    write_csv(out_root / "detection_latency_per_obstacle.csv", detection_rows)
    write_json(out_root / "confusion_matrix.json", confusion)
    write_json(out_root / "path_collision_monitor_metrics_summary.json", summary)
    print(
        "[path_collision_monitor_metrics] "
        f"cases={len(cases)} p99={latency['latency_ms_p99']:.6f} pass={summary['pass']} out={out_root}"
    )
    return 0 if summary["pass"] else 1


def synthetic_cases(*, frames: int, seed: int) -> List[SyntheticCase]:
    rng = np.random.default_rng(seed)
    rows: List[SyntheticCase] = []
    insertion_frame = max(3, int(frames) // 3)
    for frame in range(int(frames)):
        clear_q = np.zeros((4, 3), dtype=np.float64)
        clear_q[:, 0] = rng.uniform(0.0, 0.2, size=4)
        if frame < insertion_frame:
            rows.append(SyntheticCase("true_negative_clear", frame, clear_q, False, False))
        elif frame < insertion_frame + 2:
            # Ground truth says the obstacle has entered the swept tube, but the
            # monitor reference has not yet intersected it: a controlled miss.
            rows.append(SyntheticCase("missed_detection_warmup", frame, clear_q, True, True))
        else:
            hit_q = clear_q.copy()
            hit_q[2, 0] = 0.8
            rows.append(SyntheticCase("true_positive_collision", frame, hit_q, True, True))

    fp_q = np.zeros((4, 3), dtype=np.float64)
    fp_q[1, 0] = 0.9
    rows.append(SyntheticCase("false_positive_synthetic_oracle", int(frames), fp_q, False, True))
    return rows


def evaluate_cases(cases: Sequence[SyntheticCase], *, iterations: int) -> tuple[List[Dict[str, Any]], List[float]]:
    monitor = PathCollisionMonitor(SyntheticCapsule(), handle=SyntheticHandle())
    blocked = np.zeros((8, 8), dtype=bool)
    rows: List[Dict[str, Any]] = []
    latency_samples: List[float] = []
    repeat = max(1, int(iterations))
    for case in cases:
        report = None
        elapsed_ms = 0.0
        for _ in range(repeat):
            t0 = time.perf_counter()
            report = monitor.check({"q_traj": case.q_traj}, blocked)
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            latency_samples.append(float(elapsed_ms))
        assert report is not None
        rows.append(
            {
                "case_id": case.case_id,
                "frame_index": int(case.frame_index),
                "obstacle_inserted": bool(case.obstacle_inserted),
                "truth_collision": bool(case.truth_collision),
                "reported_collision": bool(report.is_collision),
                "colliding_cells_count": int(report.colliding_cells_count),
                "checked_sample_count": int(report.checked_sample_count),
                "last_call_latency_ms": float(elapsed_ms),
            }
        )
    return rows, latency_samples


def latency_distribution(samples: Sequence[float]) -> Dict[str, Any]:
    arr = np.asarray(samples, dtype=np.float64)
    return {
        "sample_count": int(arr.size),
        "latency_ms_p50": float(np.percentile(arr, 50.0)) if arr.size else 0.0,
        "latency_ms_p95": float(np.percentile(arr, 95.0)) if arr.size else 0.0,
        "latency_ms_p99": float(np.percentile(arr, 99.0)) if arr.size else 0.0,
        "latency_ms_max": float(np.max(arr)) if arr.size else 0.0,
    }


def detection_latency_rows(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    inserted = [r for r in rows if bool(r["obstacle_inserted"])]
    if not inserted:
        return []
    first_insert = min(int(r["frame_index"]) for r in inserted)
    reported = [r for r in inserted if bool(r["reported_collision"])]
    if not reported:
        return [
            {
                "obstacle_id": "synthetic_obstacle_0",
                "inserted_frame": first_insert,
                "first_detection_frame": None,
                "detection_latency_frames": None,
            }
        ]
    first_detection = min(int(r["frame_index"]) for r in reported)
    return [
        {
            "obstacle_id": "synthetic_obstacle_0",
            "inserted_frame": int(first_insert),
            "first_detection_frame": int(first_detection),
            "detection_latency_frames": int(first_detection - first_insert),
        }
    ]


def confusion_matrix(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    tp = fp = tn = fn = 0
    for row in rows:
        truth = bool(row["truth_collision"])
        pred = bool(row["reported_collision"])
        if truth and pred:
            tp += 1
        elif truth and not pred:
            fn += 1
        elif (not truth) and pred:
            fp += 1
        else:
            tn += 1
    precision = safe_div(tp, tp + fp)
    recall = safe_div(tp, tp + fn)
    return {
        "tp": int(tp),
        "fp": int(fp),
        "tn": int(tn),
        "fn": int(fn),
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(safe_div(2.0 * precision * recall, precision + recall)),
        "false_positive_rate": float(safe_div(fp, fp + tn)),
        "missed_detection_rate": float(safe_div(fn, fn + tp)),
    }


def safe_div(num: float, den: float) -> float:
    return float(num / den) if den else 0.0


def write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    keys: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, data: Mapping[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


if __name__ == "__main__":
    raise SystemExit(main())
