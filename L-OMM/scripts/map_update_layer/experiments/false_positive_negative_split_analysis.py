#!/usr/bin/env python3
"""Baseline vs synthetic-overlay classifier stability analysis."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, List, Mapping

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))

try:
    from control_module.capsule_collision import CapsuleCollision
    from control_module.constants import DEFAULT_MU_MIN
    from control_module.path_planning import PathPlanning
except (ImportError, ModuleNotFoundError):
    from capsule_collision import CapsuleCollision
    from constants import DEFAULT_MU_MIN
    from path_planning import PathPlanning

from map_update_layer.active_map_snapshot import ActiveMapSnapshot
from map_update_layer.blockage_classifier import ReferenceBlockageClassifier
from map_update_layer.path_collision_monitor import PathCollisionMonitor


def run_analysis(*, snapshots: str, out: str, map_path: str | None = None) -> Dict[str, Any]:
    snapshot_root = Path(snapshots)
    out_root = Path(out)
    out_root.mkdir(parents=True, exist_ok=True)

    planner = PathPlanning(map_path=map_path)
    handle = planner.load_map(map_path)
    ref_npz = np.load(snapshot_root / "accepted_reference.npz", allow_pickle=False)
    reference = {
        "s": np.asarray(ref_npz["s"], dtype=np.float64),
        "xz": np.asarray(ref_npz["xz"], dtype=np.float64).reshape(-1, 2),
    }
    q_traj = np.asarray(ref_npz["q_traj"], dtype=np.float64).reshape(-1, 3)
    accepted_reference: Dict[str, Any] = {"xz_path": reference["xz"]}
    if q_traj.shape[0] == reference["xz"].shape[0] and q_traj.shape[0] > 0:
        accepted_reference["q_traj"] = q_traj
    initial_blocked = np.asarray(ref_npz["initial_blocked_mask"], dtype=bool)
    initial_final = np.asarray(ref_npz["initial_final_active_mask"], dtype=bool)

    classifier = ReferenceBlockageClassifier(
        path_collision_monitor=PathCollisionMonitor(CapsuleCollision(), handle=handle)
    )
    rows: List[Dict[str, Any]] = []
    for idx, frame_path in enumerate(sorted((snapshot_root / "frames").glob("frame_*.npz"))):
        data = np.load(frame_path, allow_pickle=False)
        blocked = np.asarray(data["blocked_mask"], dtype=bool)
        occupied = np.asarray(data["occupied_mask"], dtype=bool) if "occupied_mask" in data.files else blocked.copy()
        final = np.asarray(data["final_active_mask"], dtype=bool) if "final_active_mask" in data.files else (initial_final & (~blocked))
        baseline_snapshot = _snapshot(handle, blocked, final, occupied)
        rows.append(_classify_row(classifier, baseline_snapshot, reference, accepted_reference, initial_blocked, idx, "baseline"))

        overlay_blocked = blocked.copy()
        _overlay_reference_region(handle, overlay_blocked, reference["xz"])
        overlay_snapshot = _snapshot(handle, overlay_blocked, initial_final & (~overlay_blocked), overlay_blocked)
        rows.append(_classify_row(classifier, overlay_snapshot, reference, accepted_reference, initial_blocked, idx, "synthetic_overlay"))

    _write_csv(out_root / "per_frame_breakdown.csv", rows)
    baseline = [r for r in rows if r["scenario"] == "baseline"]
    overlay = [r for r in rows if r["scenario"] == "synthetic_overlay"]
    false_positive_count = sum(1 for r in baseline if r["event_class"] == "REFERENCE_BLOCKED")
    false_negative_count = sum(1 for r in overlay if r["event_class"] != "REFERENCE_BLOCKED")
    summary = {
        "baseline_scenario_evaluated": bool(baseline),
        "overlay_scenario_evaluated": bool(overlay),
        "baseline_frame_count": int(len(baseline)),
        "overlay_frame_count": int(len(overlay)),
        "false_positive_count": int(false_positive_count),
        "false_positive_rate": float(false_positive_count / max(len(baseline), 1)),
        "false_negative_count": int(false_negative_count),
        "false_negative_rate": float(false_negative_count / max(len(overlay), 1)),
        "baseline_event_counts": _event_counts(baseline),
        "overlay_event_counts": _event_counts(overlay),
        "robot_command_endpoint_touched": False,
    }
    _write_json(out_root / "false_positive_negative_summary.json", summary)
    return summary


def _snapshot(handle: Any, blocked: np.ndarray, final: np.ndarray, occupied: np.ndarray) -> ActiveMapSnapshot:
    return ActiveMapSnapshot(
        handle=handle,
        base_feasible_mask=np.ones(handle.shape, dtype=bool),
        final_active_mask=np.asarray(final, dtype=bool),
        blocked_mask=np.asarray(blocked, dtype=bool),
        occupied_mask=np.asarray(occupied, dtype=bool),
    )


def _classify_row(
    classifier: ReferenceBlockageClassifier,
    snapshot: ActiveMapSnapshot,
    reference: Mapping[str, np.ndarray],
    accepted_reference: Mapping[str, Any],
    previous_blocked: np.ndarray,
    frame_id: int,
    scenario: str,
) -> Dict[str, Any]:
    report = classifier.classify(
        snapshot,
        s_table=reference["s"],
        xz_table=reference["xz"],
        current_s_m=0.0,
        previous_blocked_mask=previous_blocked,
        accepted_reference=accepted_reference,
    )
    data = report.as_dict()
    return {
        "frame_id": int(frame_id),
        "scenario": str(scenario),
        "event_class": str(report.event_class),
        "replanning_triggered": bool(report.replanning_triggered),
        "reason": str(report.reason),
        "reference_blocked_source": str(data.get("reference_blocked_source", "none")),
        "reference_blocked_count": int(report.reference_blocked_count),
        "path_collision_is_collision": bool(data.get("path_collision_is_collision", False)),
    }


def _overlay_reference_region(handle: Any, blocked: np.ndarray, xz_path: np.ndarray) -> None:
    pts = np.asarray(xz_path, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] == 0:
        return
    pt = pts[pts.shape[0] // 2]
    cx = int(round((float(pt[0]) - float(handle.x0)) / float(handle.resolution_m)))
    cz = int(round((float(pt[1]) - float(handle.z0)) / float(handle.resolution_m)))
    n_x, n_z = blocked.shape
    radius = 2
    for ix in range(max(0, cx - radius), min(n_x, cx + radius + 1)):
        for iz in range(max(0, cz - radius), min(n_z, cz + radius + 1)):
            blocked[ix, iz] = True


def _event_counts(rows: Iterable[Mapping[str, Any]]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for row in rows:
        key = str(row.get("event_class", "UNKNOWN"))
        counts[key] = counts.get(key, 0) + 1
    return counts


def _write_json(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    rows = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(dict(row))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshots", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--map-path", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = run_analysis(snapshots=args.snapshots, out=args.out, map_path=args.map_path)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
