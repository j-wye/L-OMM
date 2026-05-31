#!/usr/bin/env python3
"""Attribute live goal-corridor blockage from rich passive-decision artifacts."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Sequence

import numpy as np


REQUIRED_FRAME_MASKS = (
    "blocked_mask",
    "final_active_mask",
    "base_feasible_mask",
    "occupied_mask",
    "fov_mask",
    "layer_blocked",
    "layer_occupied",
    "layer_target",
    "layer_unknown",
    "layer_occluded",
    "layer_inflated",
    "layer_sensor_inflated",
    "layer_occlusion_inflated",
)

SOURCE_MASKS = {
    "occupied": "layer_occupied",
    "target": "layer_target",
    "unknown": "layer_unknown",
    "occluded": "layer_occluded",
    "inflated": "layer_inflated",
    "sensor_inflated": "layer_sensor_inflated",
    "occlusion_inflated": "layer_occlusion_inflated",
}

SOURCE_PRIORITY = (
    "occupied",
    "target",
    "unknown",
    "occluded",
    "sensor_inflated",
    "occlusion_inflated",
    "inflated",
    "base_feasible_exclusion",
    "fov_exclusion",
    "residual_unattributed",
)


class MissingFieldError(RuntimeError):
    """Raised when a saved run is too thin for source attribution."""


def analyze_run(run_dir: Path, out_dir: Path) -> Dict[str, Any]:
    run_dir = Path(run_dir)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    events_path = run_dir / "events.csv"
    if not events_path.exists():
        raise MissingFieldError(f"missing events.csv: {events_path}")

    rows = _read_csv(events_path)
    if not rows:
        raise MissingFieldError(f"empty events.csv: {events_path}")

    attribution_rows = []
    for event_row in rows:
        frame_id = int(event_row.get("frame_id", len(attribution_rows)))
        frame_artifact = str(event_row.get("frame_artifact", f"frame_{frame_id:04d}.npz"))
        metadata_artifact = str(event_row.get("frame_metadata_artifact", frame_artifact.replace(".npz", ".json")))
        frame_path = run_dir / "frames" / frame_artifact
        metadata_path = run_dir / "frames" / metadata_artifact
        frame_data = _load_required_npz(frame_path)
        metadata = _load_required_metadata(metadata_path)
        goal_diag = metadata.get("goal_diagnostics")
        if not isinstance(goal_diag, Mapping):
            raise MissingFieldError(f"{metadata_path} missing goal_diagnostics")

        goal_cells = _coerce_cells(goal_diag.get("goal_corridor_cells"), "goal_corridor_cells", metadata_path)
        status_cells = _coerce_cells(
            goal_diag.get("goal_status_neighborhood_cells"),
            "goal_status_neighborhood_cells",
            metadata_path,
        )
        current_cells = _coerce_cells(goal_diag.get("current_pose_cells"), "current_pose_cells", metadata_path)
        goal_counts = _count_sources(frame_data, goal_cells)
        status_counts = _count_sources(frame_data, status_cells)
        current_counts = _count_sources(frame_data, current_cells)
        dominant_source = _dominant_source(goal_counts)
        attribution_rows.append(
            {
                "frame_id": frame_id,
                "event_class": str(event_row.get("event_class", "")),
                "reason": str(event_row.get("reason", "")),
                "dominant_goal_corridor_source": dominant_source,
                "goal_corridor_cell_count": goal_counts["cell_count"],
                "goal_corridor_blocked_count": goal_counts["blocked"],
                "goal_corridor_occupied_count": goal_counts["occupied"],
                "goal_corridor_target_count": goal_counts["target"],
                "goal_corridor_unknown_count": goal_counts["unknown"],
                "goal_corridor_occluded_count": goal_counts["occluded"],
                "goal_corridor_inflated_count": goal_counts["inflated"],
                "goal_corridor_sensor_inflated_count": goal_counts["sensor_inflated"],
                "goal_corridor_occlusion_inflated_count": goal_counts["occlusion_inflated"],
                "goal_corridor_base_feasible_exclusion_count": goal_counts["base_feasible_exclusion"],
                "goal_corridor_fov_exclusion_count": goal_counts["fov_exclusion"],
                "goal_corridor_residual_unattributed_count": goal_counts["residual_unattributed"],
                "goal_status_blocked_count": status_counts["blocked"],
                "goal_status_final_active_count": status_counts["final_active"],
                "current_pose_blocked_count": current_counts["blocked"],
                "current_pose_final_active_count": current_counts["final_active"],
                "fov_mask_available": bool(_nested_get(metadata, ("fov_stats", "fov_mask_available"), False)),
                "frame_artifact": frame_artifact,
                "frame_metadata_artifact": metadata_artifact,
            }
        )

    out_csv = out_dir / "source_attribution.csv"
    _write_csv(out_csv, attribution_rows)
    summary = {
        "run_dir": str(run_dir),
        "frames_analyzed": len(attribution_rows),
        "source_counts": _histogram(row["dominant_goal_corridor_source"] for row in attribution_rows),
        "all_goal_corridor_blocked": all(int(row["goal_corridor_blocked_count"]) > 0 for row in attribution_rows),
        "all_no_command_markers_present": all(_metadata_no_command(run_dir, row) for row in attribution_rows),
        "output_csv": str(out_csv),
    }
    _write_json(out_dir / "summary.json", summary)
    return summary


def _load_required_npz(path: Path) -> Mapping[str, np.ndarray]:
    if not path.exists():
        raise MissingFieldError(f"missing frame artifact: {path}")
    data = np.load(path, allow_pickle=False)
    missing = [name for name in REQUIRED_FRAME_MASKS if name not in data.files]
    if missing:
        raise MissingFieldError(f"{path} missing required masks: {', '.join(missing)}")
    return {name: np.asarray(data[name], dtype=bool) for name in data.files if name.endswith("_mask") or name.startswith("layer_")}


def _load_required_metadata(path: Path) -> Mapping[str, Any]:
    if not path.exists():
        raise MissingFieldError(f"missing frame metadata: {path}")
    with path.open("r", encoding="utf-8") as f:
        metadata = json.load(f)
    required = (
        "robot_command_endpoint_touched",
        "control_module_called_in_live_loop",
        "map_geometry",
        "snapshot_stats",
        "result_stats",
        "classifier_report",
        "goal_diagnostics",
        "fov_stats",
    )
    missing = [name for name in required if name not in metadata]
    if missing:
        raise MissingFieldError(f"{path} missing required metadata: {', '.join(missing)}")
    return metadata


def _coerce_cells(value: Any, name: str, path: Path) -> list[tuple[int, int]]:
    if not isinstance(value, Sequence):
        raise MissingFieldError(f"{path} missing {name}")
    cells = []
    for cell in value:
        if not isinstance(cell, Sequence) or len(cell) != 2:
            raise MissingFieldError(f"{path} has invalid {name} entry: {cell!r}")
        cells.append((int(cell[0]), int(cell[1])))
    return cells


def _count_sources(frame_data: Mapping[str, np.ndarray], cells: Sequence[tuple[int, int]]) -> Dict[str, int]:
    counts: Dict[str, int] = {
        "cell_count": int(len(cells)),
        "blocked": _count(frame_data["blocked_mask"], cells),
        "final_active": _count(frame_data["final_active_mask"], cells),
        "base_feasible": _count(frame_data["base_feasible_mask"], cells),
        "fov": _count(frame_data["fov_mask"], cells),
    }
    known_union = np.zeros_like(frame_data["blocked_mask"], dtype=bool)
    for source_name, mask_name in SOURCE_MASKS.items():
        counts[source_name] = _count(frame_data[mask_name], cells)
        known_union |= frame_data[mask_name]
    counts["base_feasible_exclusion"] = int(
        sum(1 for ix, iz in cells if not bool(frame_data["base_feasible_mask"][ix, iz]))
    )
    counts["fov_exclusion"] = int(
        sum(1 for ix, iz in cells if not bool(frame_data["fov_mask"][ix, iz]))
    )
    counts["residual_unattributed"] = int(
        sum(
            1
            for ix, iz in cells
            if bool(frame_data["blocked_mask"][ix, iz]) and not bool(known_union[ix, iz])
        )
    )
    return counts


def _dominant_source(counts: Mapping[str, int]) -> str:
    best = max(SOURCE_PRIORITY, key=lambda name: int(counts.get(name, 0)))
    return best if int(counts.get(best, 0)) > 0 else "none"


def _count(mask: np.ndarray, cells: Sequence[tuple[int, int]]) -> int:
    return int(sum(1 for ix, iz in cells if bool(mask[ix, iz])))


def _metadata_no_command(run_dir: Path, row: Mapping[str, Any]) -> bool:
    metadata_path = run_dir / "frames" / str(row["frame_metadata_artifact"])
    with metadata_path.open("r", encoding="utf-8") as f:
        metadata = json.load(f)
    return (
        metadata.get("robot_command_endpoint_touched") is False
        and metadata.get("control_module_called_in_live_loop") is False
    )


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


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


def _write_json(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _histogram(values: Iterable[Any]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for value in values:
        key = str(value)
        counts[key] = counts.get(key, 0) + 1
    return counts


def _nested_get(data: Mapping[str, Any], path: Sequence[str], default: Any) -> Any:
    cur: Any = data
    for key in path:
        if not isinstance(cur, Mapping):
            return default
        cur = cur.get(key)
    return default if cur is None else cur


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--out", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out)
    try:
        summary = analyze_run(Path(args.run_dir), out_dir)
    except MissingFieldError as exc:
        out_dir.mkdir(parents=True, exist_ok=True)
        _write_json(
            out_dir / "source_attribution_error.json",
            {
                "status": "MISSING_REQUIRED_FIELD",
                "error": str(exc),
                "required_frame_masks": list(REQUIRED_FRAME_MASKS),
            },
        )
        raise SystemExit(str(exc))
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
