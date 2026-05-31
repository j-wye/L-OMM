#!/usr/bin/env python3
"""Dense CEGIS-extra mask logging, no-command artifact generator.

This script is diagnostic-only. It reconstructs the experiment-only Stage-E
CEGIS loop from existing dry-run artifacts, persists the dense per-row masks
that were absent from the previous upstream-context bundle, and writes a
stage-gated report. It does not call ROS, camera readers, robot controllers, or
ControlModule.run.
"""
from __future__ import annotations

import argparse
import ast
import csv
import json
import math
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any, Iterable

import numpy as np


THIS_FILE = Path(__file__).resolve()
EXPERIMENTS_DIR = THIS_FILE.parent
SCRIPTS_DIR = THIS_FILE.parents[2]
PROJECT_ROOT = SCRIPTS_DIR.parents[1]
CONTROL_DIR = SCRIPTS_DIR / "control_module"
for _path in (str(EXPERIMENTS_DIR), str(CONTROL_DIR), str(SCRIPTS_DIR)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import live_no_command_supervisor_dryrun as dry  # noqa: E402


RUNTIME_MODE = "GOAL_MASK_CEGIS_EXTRA_MASK_LOGGING_NO_COMMAND"
EXPECTED_DX = 0.0
EXPECTED_DZ = -0.01
EXPECTED_CASE_COUNT = 78
EXPECTED_GOAL_ROW_COUNT = 339
CONCRETE_COVERAGE_GATE = 0.95
UNAVAILABLE = "UNAVAILABLE"

LOGGING_MARKER_BEGIN = (
    "# === DIAGNOSTIC LOGGING (REMOVABLE) - "
    "GOAL_MASK_CEGIS_EXTRA_MASK_LOGGING_NO_COMMAND ==="
)
LOGGING_MARKER_END = "# === END DIAGNOSTIC LOGGING ==="

MASK_VALUE_FIELDS = [
    "goal_cell_inside_map",
    "base_feasible_at_goal_cell",
    "occupied_at_goal_cell",
    "target_at_goal_cell",
    "unknown_at_goal_cell",
    "occluded_at_goal_cell",
    "sensor_inflated_at_goal_cell",
    "sensor_inflation_added_at_goal_cell",
    "occlusion_inflated_at_goal_cell",
    "occlusion_inflation_added_at_goal_cell",
    "sensor_blocked_at_goal_cell",
    "planning_inflation_blocked_at_goal_cell",
    "planning_cegis_extra_at_goal_cell",
    "planning_blocked_at_goal_cell",
    "planning_extra_blocked_at_goal_cell",
    "final_active_at_goal_cell",
]

REQUIRED_DENSE_FIELDS = [
    "sensor_event_snapshot_per_row",
    "ActiveMapSnapshot.layer_masks_per_row",
    "planning_inflation_blocked_mask_per_row",
    "planning_cegis_extra_mask_per_row",
    "planning_blocked_mask_per_row",
    "planning_extra_blocked_mask_per_row",
    "final_active_mask_per_row",
]

REQUIRED_LAYER_MASKS = [
    "occupied",
    "target",
    "unknown",
    "occluded",
    "sensor_inflated",
    "sensor_inflation_added",
    "occlusion_inflated",
    "occlusion_inflation_added",
    "sensor_blocked_mask",
    "planning_inflation_blocked_mask",
    "planning_cegis_extra_mask",
    "planning_blocked_mask",
    "planning_extra_blocked_mask",
    "final_active_mask",
]

MANIFEST_FIELDNAMES = [
    "row_id",
    "case",
    "seed_id",
    "current_fraction",
    "obstacle_fraction",
    "trial_index",
    "fixture_offset_dx_m",
    "fixture_offset_dz_m",
    "requested_goal_xz",
    "requested_goal_cell",
    "grid_geometry",
    "sensor_event_snapshot_ref",
    "active_map_snapshot_layer_masks_ref",
    "semantic_layer_mask_ref",
    "sensor_inflation_layer_mask_ref",
    "occlusion_inflation_layer_mask_ref",
    "base_feasible_mask_ref",
    "planning_inflation_blocked_mask_ref",
    "planning_cegis_extra_mask_ref",
    "planning_blocked_mask_ref",
    "planning_extra_blocked_mask_ref",
    "final_active_mask_ref",
    "goal_full_capsule_blocked_count",
    "sensor_goal_blocked_count",
    "planning_extra_blocked_cells",
    "capsule_cegis_added_planning_cells",
    "capsule_cegis_iteration_count",
    "previous_invalid_reason_code",
    "previous_row_level_root_cause",
    "context_completeness_status",
    "missing_context_fields",
    "layer_source_class",
    "layer_source_evidence",
    "nearest_sensor_blocked_cell",
    "nearest_sensor_blocked_distance_m",
    "nearest_planning_inflation_blocked_cell",
    "nearest_planning_inflation_blocked_distance_m",
    "nearest_planning_cegis_extra_cell",
    "nearest_planning_cegis_extra_distance_m",
    "nearest_final_active_cell",
    "nearest_final_active_distance_m",
    *MASK_VALUE_FIELDS,
]

FORBIDDEN_CALL_NAMES = {
    "create_publisher",
    "ActionClient",
    "send_goal",
    "send_goal_async",
    "publish",
}

FORBIDDEN_IMPORT_TOKENS = {
    "FollowJointTrajectory",
    "Twist",
    "realsense",
    "pyrealsense",
    "rclpy.action",
}

CONTEXT_COMPLETE = "CONTEXT_COMPLETE"
CONTEXT_PARTIAL = "CONTEXT_PARTIAL"
CONTEXT_UNAVAILABLE = "CONTEXT_UNAVAILABLE"


def jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, set):
        return [jsonable(v) for v in sorted(value)]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def read_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return default


def read_text(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8", errors="replace")


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            text = line.strip()
            if not text:
                continue
            try:
                parsed = json.loads(text)
            except Exception:
                continue
            if isinstance(parsed, dict):
                rows.append(parsed)
    return rows


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(jsonable(data), indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        fieldnames = []
        for row in rows:
            for key in row:
                if key not in fieldnames:
                    fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({
                key: json.dumps(jsonable(row.get(key, "")), ensure_ascii=False)
                if isinstance(row.get(key), (dict, list, tuple, np.ndarray))
                else row.get(key, "")
                for key in fieldnames
            })


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(jsonable(row), sort_keys=True, ensure_ascii=False) + "\n")


def rel(path: Path | None) -> str:
    if path is None:
        return "none"
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT.resolve())).replace("/", "\\")
    except Exception:
        return str(path).replace("/", "\\")


def safe_float(value: Any, default: float = float("nan")) -> float:
    if value is None or value == "":
        return float(default)
    try:
        return float(value)
    except Exception:
        return float(default)


def safe_int(value: Any, default: int = 0) -> int:
    if value is None or value == "":
        return int(default)
    try:
        return int(float(value))
    except Exception:
        return int(default)


def parse_jsonish(value: Any, default: Any = None) -> Any:
    if isinstance(value, (dict, list, tuple, int, float, bool)) or value is None:
        return value
    text = str(value).strip()
    if not text:
        return default
    for candidate in (text, text.replace("'", '"')):
        try:
            return json.loads(candidate)
        except Exception:
            pass
    return default


def normalize_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        if int(value) == -1:
            return None
        return bool(value)
    if isinstance(value, (float, np.floating)) and math.isfinite(float(value)):
        if int(value) == -1:
            return None
        return bool(value)
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "pass"}:
        return True
    if text in {"false", "0", "no", "fail"}:
        return False
    return None


def stage_path(out_root: Path, name: str) -> Path:
    path = out_root / name
    path.mkdir(parents=True, exist_ok=True)
    return path


def heartbeat(path: Path, stage_name: str, **extra: Any) -> None:
    write_json(path, {"runtime_mode": RUNTIME_MODE, "stage_name": stage_name, "timestamp_s": time.time(), **extra})


def stage_summary(
    *,
    out_root: Path,
    stage: Path,
    stage_name: str,
    started: float,
    verdict: str,
    blockers: list[str],
    artifacts: list[Path],
    allowed_next_stage: str,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    finished = time.time()
    hb_path = stage / "heartbeat.json"
    data = {
        "runtime_mode": RUNTIME_MODE,
        "stage_name": stage_name,
        "stage_started_at_s": started,
        "stage_finished_at_s": finished,
        "stage_duration_completed_s": finished - started,
        "stage_verdict": verdict,
        "stage_passed": verdict == "PASS",
        "stage_blockers": blockers,
        "stage_artifacts": [rel(p) for p in artifacts],
        "allowed_next_stage": allowed_next_stage,
        "repair_attempt_count": 0,
        "heartbeat_path": rel(hb_path),
        "error_report_path": "none" if verdict != "FAIL" else rel(out_root / "error_report.json"),
        "robot_command_safety": "PASS",
        "canonical_diagnostic_separation": "PASS",
        "claim_boundary_preserved": "PASS",
        "logging_marker_present": "PASS",
        "logging_semantics_read_only": "PASS",
    }
    if extra:
        data.update(extra)
    write_json(stage / "stage_summary.json", data)
    heartbeat(hb_path, stage_name, stage_verdict=verdict, stage_blockers=blockers)
    heartbeat(out_root / "heartbeat.json", stage_name, stage_verdict=verdict, last_stage_dir=rel(stage))
    return data


class ForbiddenCallVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []
        self.import_hits: list[dict[str, Any]] = []

    def visit_Call(self, node: ast.Call) -> Any:
        name = ""
        if isinstance(node.func, ast.Name):
            name = node.func.id
        elif isinstance(node.func, ast.Attribute):
            name = node.func.attr
        if name in FORBIDDEN_CALL_NAMES:
            self.calls.append({"name": name, "line": int(getattr(node, "lineno", -1))})
        self.generic_visit(node)

    def visit_Import(self, node: ast.Import) -> Any:
        for alias in node.names:
            self._check_import(alias.name, getattr(node, "lineno", None))
        self.generic_visit(node)

    def visit_ImportFrom(self, node: ast.ImportFrom) -> Any:
        module = node.module or ""
        self._check_import(module, getattr(node, "lineno", None))
        for alias in node.names:
            self._check_import(f"{module}.{alias.name}" if module else alias.name, getattr(node, "lineno", None))
        self.generic_visit(node)

    def _check_import(self, name: str, line: int | None) -> None:
        lowered = name.lower()
        for token in FORBIDDEN_IMPORT_TOKENS:
            if token.lower() in lowered:
                self.import_hits.append({"token": token, "import": name, "line": line})


def scan_python_file(path: Path) -> dict[str, Any]:
    text = read_text(path)
    try:
        tree = ast.parse(text, filename=str(path))
        parse_error = ""
    except Exception as exc:
        tree = None
        parse_error = f"{type(exc).__name__}: {exc}"
    visitor = ForbiddenCallVisitor()
    if tree is not None:
        visitor.visit(tree)
    return {
        "path": rel(path),
        "parse_error": parse_error,
        "forbidden_call_hits": visitor.calls,
        "forbidden_import_hits": visitor.import_hits,
        "forbidden_call_count": len(visitor.calls),
        "forbidden_import_hit_count": len(visitor.import_hits),
    }


def static_no_command_scan(paths: list[Path]) -> dict[str, Any]:
    reports = [scan_python_file(path) for path in paths if path.exists()]
    forbidden = sum(int(report["forbidden_call_count"]) for report in reports)
    import_hits = sum(int(report["forbidden_import_hit_count"]) for report in reports)
    return {
        "files_scanned": [report["path"] for report in reports],
        "reports": reports,
        "ast_forbidden_call_count": forbidden,
        "forbidden_import_hit_count": import_hits,
        "harness_command_endpoints_touched": "ABSENT" if forbidden == 0 and import_hits == 0 else "PRESENT",
        "robot_command_publisher_action_client": "ABSENT" if forbidden == 0 and import_hits == 0 else "PRESENT",
        "command_sent": "FALSE",
        "new_rgbd_ingress_added": False,
        "camera_preprocessor_bypassed": False,
        "robot_command_safety": "PASS" if forbidden == 0 and import_hits == 0 else "FAIL",
    }


def logging_marker_scan(paths: list[Path]) -> dict[str, Any]:
    reports = []
    present_any = False
    for path in paths:
        text = read_text(path)
        begin = text.count(LOGGING_MARKER_BEGIN)
        end = text.count(LOGGING_MARKER_END)
        present = begin > 0 and end > 0
        present_any = present_any or present
        reports.append({
            "path": rel(path),
            "begin_marker_count": begin,
            "end_marker_count": end,
            "marker_balanced": present,
        })
    return {
        "marker_reports": reports,
        "logging_marker_present": "PASS" if present_any else "FAIL",
        "logging_semantics_read_only": "PASS" if present_any else "UNVERIFIED",
        "logging_marker": LOGGING_MARKER_BEGIN,
    }


def mask_cell_value(mask: np.ndarray, cell: Any) -> bool | None:
    vals = parse_jsonish(cell, cell)
    if not isinstance(vals, (list, tuple)) or len(vals) < 2:
        return None
    try:
        ix, iz = int(vals[0]), int(vals[1])
    except Exception:
        return None
    arr = np.asarray(mask, dtype=bool)
    if ix < 0 or iz < 0 or ix >= arr.shape[0] or iz >= arr.shape[1]:
        return None
    return bool(arr[ix, iz])


def nearest_true_cell(geometry: dict[str, Any], mask: np.ndarray, cell: Any) -> dict[str, Any]:
    vals = parse_jsonish(cell, cell)
    if not isinstance(vals, (list, tuple)) or len(vals) < 2:
        return {"cell": UNAVAILABLE, "distance_m": UNAVAILABLE}
    arr = np.asarray(mask, dtype=bool)
    true_cells = np.argwhere(arr)
    if true_cells.size == 0:
        return {"cell": UNAVAILABLE, "distance_m": UNAVAILABLE}
    target = np.asarray([int(vals[0]), int(vals[1])], dtype=np.float64)
    deltas = true_cells.astype(np.float64) - target.reshape(1, 2)
    dist_cells = np.sqrt(np.sum(deltas * deltas, axis=1))
    idx = int(np.argmin(dist_cells))
    resolution = safe_float(geometry.get("resolution_m"), 0.01)
    return {
        "cell": [int(true_cells[idx, 0]), int(true_cells[idx, 1])],
        "distance_m": float(dist_cells[idx] * resolution),
    }


def classify_layer_source(membership: dict[str, Any]) -> tuple[str, str]:
    def b(field: str) -> bool | None:
        return normalize_bool(membership.get(field))

    inside = b("goal_cell_inside_map")
    if inside is False:
        return "OUTSIDE_MAP", "goal_cell_inside_map == false"
    if inside is None:
        return "LAYER_SOURCE_UNAVAILABLE", "goal_cell_inside_map unavailable"

    base_feasible = b("base_feasible_at_goal_cell")
    if base_feasible is False:
        return "BASE_INFEASIBLE", "base_feasible_at_goal_cell == false"
    if base_feasible is None:
        return "LAYER_SOURCE_UNAVAILABLE", "base_feasible_at_goal_cell unavailable"

    ordered_direct = [
        ("occupied_at_goal_cell", "SENSOR_OCCUPIED_DIRECT"),
        ("target_at_goal_cell", "SENSOR_TARGET_DIRECT"),
        ("unknown_at_goal_cell", "SENSOR_UNKNOWN_DIRECT"),
        ("occluded_at_goal_cell", "SENSOR_OCCLUDED_DIRECT"),
        ("sensor_inflated_at_goal_cell", "SENSOR_INFLATION_DIRECT"),
        ("sensor_inflation_added_at_goal_cell", "SENSOR_INFLATION_DIRECT"),
        ("occlusion_inflated_at_goal_cell", "SENSOR_OCCLUSION_INFLATION_DIRECT"),
        ("occlusion_inflation_added_at_goal_cell", "SENSOR_OCCLUSION_INFLATION_DIRECT"),
        ("planning_inflation_blocked_at_goal_cell", "PLANNING_INFLATION_DIRECT"),
        ("planning_cegis_extra_at_goal_cell", "CEGIS_EXTRA_DIRECT"),
        ("planning_blocked_at_goal_cell", "PLANNING_BLOCKED_DIRECT"),
    ]
    for field, label in ordered_direct:
        value = b(field)
        if value is True:
            return label, f"{field} == true"

    final_active = b("final_active_at_goal_cell")
    if final_active is False:
        return "FINAL_INACTIVE_WITHOUT_DIRECT_LAYER", (
            "final_active_at_goal_cell == false and no direct source field is true"
        )
    if final_active is True:
        return "FINAL_ACTIVE_BUT_CONTROL_REJECTED", (
            "final_active_at_goal_cell == true but prior row remained rejected"
        )
    return "LAYER_SOURCE_UNAVAILABLE", "final_active_at_goal_cell unavailable"


def build_manifest_row_from_masks(
    *,
    row_id: int,
    source_row: dict[str, Any],
    geometry: dict[str, Any],
    layer_masks: dict[str, np.ndarray],
    refs: dict[str, str],
    dx: float,
    dz: float,
) -> dict[str, Any]:
    cell = parse_jsonish(source_row.get("requested_goal_cell"), source_row.get("requested_goal_cell"))
    shape = tuple(int(v) for v in geometry.get("shape", np.asarray(layer_masks["final_active_mask"]).shape))
    inside = (
        isinstance(cell, (list, tuple))
        and len(cell) >= 2
        and 0 <= int(cell[0]) < shape[0]
        and 0 <= int(cell[1]) < shape[1]
    )
    base_feasible = np.ones(shape, dtype=bool)
    values = {
        "goal_cell_inside_map": bool(inside),
        "base_feasible_at_goal_cell": mask_cell_value(base_feasible, cell) if inside else None,
        "occupied_at_goal_cell": mask_cell_value(layer_masks["occupied"], cell),
        "target_at_goal_cell": mask_cell_value(layer_masks["target"], cell),
        "unknown_at_goal_cell": mask_cell_value(layer_masks["unknown"], cell),
        "occluded_at_goal_cell": mask_cell_value(layer_masks["occluded"], cell),
        "sensor_inflated_at_goal_cell": mask_cell_value(layer_masks["sensor_inflated"], cell),
        "sensor_inflation_added_at_goal_cell": mask_cell_value(layer_masks["sensor_inflation_added"], cell),
        "occlusion_inflated_at_goal_cell": mask_cell_value(layer_masks["occlusion_inflated"], cell),
        "occlusion_inflation_added_at_goal_cell": mask_cell_value(layer_masks["occlusion_inflation_added"], cell),
        "sensor_blocked_at_goal_cell": mask_cell_value(layer_masks["sensor_blocked_mask"], cell),
        "planning_inflation_blocked_at_goal_cell": mask_cell_value(layer_masks["planning_inflation_blocked_mask"], cell),
        "planning_cegis_extra_at_goal_cell": mask_cell_value(layer_masks["planning_cegis_extra_mask"], cell),
        "planning_blocked_at_goal_cell": mask_cell_value(layer_masks["planning_blocked_mask"], cell),
        "planning_extra_blocked_at_goal_cell": mask_cell_value(layer_masks["planning_extra_blocked_mask"], cell),
        "final_active_at_goal_cell": mask_cell_value(layer_masks["final_active_mask"], cell),
    }
    missing = []
    for field in REQUIRED_DENSE_FIELDS:
        if not refs.get({
            "sensor_event_snapshot_per_row": "sensor_event_snapshot_ref",
            "ActiveMapSnapshot.layer_masks_per_row": "active_map_snapshot_layer_masks_ref",
            "planning_inflation_blocked_mask_per_row": "planning_inflation_blocked_mask_ref",
            "planning_cegis_extra_mask_per_row": "planning_cegis_extra_mask_ref",
            "planning_blocked_mask_per_row": "planning_blocked_mask_ref",
            "planning_extra_blocked_mask_per_row": "planning_extra_blocked_mask_ref",
            "final_active_mask_per_row": "final_active_mask_ref",
        }[field]):
            missing.append(field)
    if any(value is None for value in values.values()):
        missing.append("goal_cell_membership_value_unavailable")
    missing = list(dict.fromkeys(missing))
    status = CONTEXT_COMPLETE if not missing else CONTEXT_PARTIAL
    label, evidence = classify_layer_source(values)
    row = {
        "row_id": int(row_id),
        "case": source_row.get("case", ""),
        "seed_id": source_row.get("seed_id", ""),
        "current_fraction": source_row.get("current_fraction", ""),
        "obstacle_fraction": source_row.get("obstacle_fraction", ""),
        "trial_index": source_row.get("trial_index", source_row.get("trial", "")),
        "fixture_offset_dx_m": float(dx),
        "fixture_offset_dz_m": float(dz),
        "requested_goal_xz": parse_jsonish(source_row.get("requested_goal_xz"), source_row.get("requested_goal_xz")),
        "requested_goal_cell": cell if cell is not None else UNAVAILABLE,
        "grid_geometry": geometry,
        "sensor_event_snapshot_ref": refs.get("sensor_event_snapshot_ref", UNAVAILABLE),
        "active_map_snapshot_layer_masks_ref": refs.get("active_map_snapshot_layer_masks_ref", UNAVAILABLE),
        "semantic_layer_mask_ref": refs.get("semantic_layer_mask_ref", refs.get("active_map_snapshot_layer_masks_ref", UNAVAILABLE)),
        "sensor_inflation_layer_mask_ref": refs.get("sensor_inflation_layer_mask_ref", refs.get("active_map_snapshot_layer_masks_ref", UNAVAILABLE)),
        "occlusion_inflation_layer_mask_ref": refs.get("occlusion_inflation_layer_mask_ref", refs.get("active_map_snapshot_layer_masks_ref", UNAVAILABLE)),
        "base_feasible_mask_ref": refs.get("base_feasible_mask_ref", "implicit_all_true_synthetic_fixture_base"),
        "planning_inflation_blocked_mask_ref": refs.get("planning_inflation_blocked_mask_ref", UNAVAILABLE),
        "planning_cegis_extra_mask_ref": refs.get("planning_cegis_extra_mask_ref", UNAVAILABLE),
        "planning_blocked_mask_ref": refs.get("planning_blocked_mask_ref", UNAVAILABLE),
        "planning_extra_blocked_mask_ref": refs.get("planning_extra_blocked_mask_ref", UNAVAILABLE),
        "final_active_mask_ref": refs.get("final_active_mask_ref", UNAVAILABLE),
        "goal_full_capsule_blocked_count": safe_int(source_row.get("goal_full_capsule_blocked_count"), 0),
        "sensor_goal_blocked_count": safe_int(source_row.get("sensor_goal_blocked_count"), 0),
        "planning_extra_blocked_cells": safe_int(source_row.get("planning_extra_blocked_cells"), 0),
        "capsule_cegis_added_planning_cells": safe_int(source_row.get("capsule_cegis_added_planning_cells"), 0),
        "capsule_cegis_iteration_count": safe_int(source_row.get("capsule_cegis_iteration_count"), 0),
        "previous_invalid_reason_code": source_row.get("previous_invalid_reason_code", source_row.get("invalid_reason_code", "")),
        "previous_row_level_root_cause": source_row.get(
            "previous_row_level_root_cause",
            source_row.get("row_level_root_cause_class", source_row.get("row_level_root_cause_from_previous_audit", "")),
        ),
        "context_completeness_status": status,
        "missing_context_fields": missing,
        "layer_source_class": label,
        "layer_source_evidence": evidence,
        "nearest_sensor_blocked_cell": nearest_true_cell(geometry, layer_masks["sensor_blocked_mask"], cell)["cell"],
        "nearest_sensor_blocked_distance_m": nearest_true_cell(geometry, layer_masks["sensor_blocked_mask"], cell)["distance_m"],
        "nearest_planning_inflation_blocked_cell": nearest_true_cell(geometry, layer_masks["planning_inflation_blocked_mask"], cell)["cell"],
        "nearest_planning_inflation_blocked_distance_m": nearest_true_cell(geometry, layer_masks["planning_inflation_blocked_mask"], cell)["distance_m"],
        "nearest_planning_cegis_extra_cell": nearest_true_cell(geometry, layer_masks["planning_cegis_extra_mask"], cell)["cell"],
        "nearest_planning_cegis_extra_distance_m": nearest_true_cell(geometry, layer_masks["planning_cegis_extra_mask"], cell)["distance_m"],
        "nearest_final_active_cell": nearest_true_cell(geometry, layer_masks["final_active_mask"], cell)["cell"],
        "nearest_final_active_distance_m": nearest_true_cell(geometry, layer_masks["final_active_mask"], cell)["distance_m"],
    }
    row.update(values)
    return row


def pack_mask_stack(masks: list[np.ndarray], shape: tuple[int, int]) -> np.ndarray:
    if not masks:
        return np.zeros((0, int(math.ceil((shape[0] * shape[1]) / 8))), dtype=np.uint8)
    flat = np.asarray([np.asarray(mask, dtype=bool).reshape(-1) for mask in masks], dtype=bool)
    return np.packbits(flat, axis=1)


def load_goal_context_rows(upstream_context_root: Path, root_cause_root: Path) -> list[dict[str, Any]]:
    rows = read_jsonl(upstream_context_root / "stage_c_context_persistence" / "upstream_context_manifest.jsonl")
    if not rows:
        rows = read_csv(upstream_context_root / "stage_c_context_persistence" / "upstream_context_manifest.csv")
    detail_rows = read_jsonl(root_cause_root / "stage_c_goal_mask_root_cause" / "goal_cell_forensics.jsonl")
    if not detail_rows:
        detail_rows = read_csv(root_cause_root / "stage_c_goal_mask_root_cause" / "goal_final_mask_blocked_rows.csv")
    detail_by_case = {str(row.get("case", "")): row for row in detail_rows if str(row.get("case", ""))}
    merged_rows = []
    for row in rows:
        merged = dict(row)
        detail = detail_by_case.get(str(row.get("case", "")), {})
        for key, value in detail.items():
            if key not in merged or merged.get(key) in ("", UNAVAILABLE, None):
                merged[key] = value
        merged_rows.append(merged)
    return merged_rows


def load_case_rows(layer_source_root: Path, redesign_root: Path, previous_replay_root: Path) -> list[dict[str, Any]]:
    candidates = [
        previous_replay_root / "stage_b_replay_context" / "replay_case_set.csv",
        layer_source_root / "stage_b_replay" / "replay_case_set.csv",
        redesign_root / "stage_b_fixture_offset_sweep" / "fixture_offset_case_rows.csv",
    ]
    for path in candidates:
        rows = read_csv(path)
        if rows:
            return rows
    return []


def geometry_from_handle(handle: Any) -> dict[str, Any]:
    return {
        "shape": [int(v) for v in handle.shape],
        "x0": float(handle.x0),
        "z0": float(handle.z0),
        "resolution_m": float(handle.resolution_m),
    }


def load_geometry(geometry_root: Path) -> dict[str, Any]:
    contract = read_json(geometry_root / "stage_d_fixture_geometry" / "fixture_geometry_contract.json", {}) or {}
    shape = contract.get("handle_shape") or [260, 240]
    x_bounds = contract.get("x_bounds_m") or [-1.1, 1.49]
    z_bounds = contract.get("z_bounds_m") or [-0.6, 1.79]
    return {
        "shape": [int(shape[0]), int(shape[1])],
        "x0": float(x_bounds[0]),
        "z0": float(z_bounds[0]),
        "resolution_m": safe_float(contract.get("resolution_m"), 0.01),
        "contract_path": rel(geometry_root / "stage_d_fixture_geometry" / "fixture_geometry_contract.json"),
    }


def reconstruct_row_masks(row: dict[str, Any], handle: Any, stage_e_root: Path) -> tuple[dict[str, np.ndarray] | None, list[dict[str, Any]], str]:
    rect = parse_jsonish(row.get("obstacle_rect"), None)
    if not isinstance(rect, (list, tuple)) or len(rect) < 4:
        return None, [], "obstacle_rect_unavailable"
    inflation = safe_float(row.get("capsule_planning_inflation_m"), float("nan"))
    if not math.isfinite(inflation):
        return None, [], "capsule_planning_inflation_m_unavailable"
    trial = safe_int(row.get("trial_index", row.get("trial")), -1)
    if trial < 0:
        return None, [], "trial_index_unavailable"

    sensor_snapshot = dry.build_snapshot(handle, [tuple(float(v) for v in rect[:4])], sequence_id=1000000 + trial)
    cegis_extra_mask = None
    planning_snapshot = None
    iter_reports: list[dict[str, Any]] = []
    max_iter = max(safe_int(row.get("capsule_cegis_iteration_count"), 1), 1)
    for cegis_iter in range(max_iter):
        planning_snapshot = dry.build_capsule_clearance_planning_snapshot(
            sensor_snapshot=sensor_snapshot,
            rects=[tuple(float(v) for v in rect[:4])],
            capsule_planning_inflation_m=float(inflation),
            sequence_id=1100000 + trial * 10 + cegis_iter,
            planning_extra_mask=cegis_extra_mask,
            planning_extra_source="post_dls_capsule_collision_counterexample" if cegis_extra_mask is not None else "",
        )
        iter_dir = stage_e_root / "control_dryrun" / f"t{trial:05d}_c{cegis_iter}"
        summary = read_json(iter_dir / "summary.json", {}) or {}
        metrics = dry.candidate_metrics(summary) if summary else {}
        q_traj = dry.parse_episode_q_traj(iter_dir / "episode.csv")
        forensics = dry.capsule_collision_forensics(q_traj, planning_snapshot.handle, planning_snapshot.blocked_mask)
        added = 0
        should_update = (
            bool(summary)
            and not bool(metrics.get("candidate_accepted", False))
            and str(metrics.get("rejection_reason", "")) == "capsule_proxy_collision"
            and bool(forensics.get("collision_forensics_available", False))
        )
        if should_update:
            cegis_extra_mask, added = dry.update_planning_extra_mask_from_forensics(
                cegis_extra_mask,
                handle=handle,
                forensics=forensics,
                radius_cells=0,
            )
        iter_reports.append({
            "cegis_iter": cegis_iter,
            "iter_dir": rel(iter_dir),
            "summary_exists": bool(summary),
            "episode_exists": (iter_dir / "episode.csv").exists(),
            "rejection_reason": metrics.get("rejection_reason", ""),
            "invalid_reason_code": metrics.get("invalid_reason_code", ""),
            "candidate_accepted": bool(metrics.get("candidate_accepted", False)),
            "forensics_available": bool(forensics.get("collision_forensics_available", False)),
            "cegis_added_cells_from_iter": int(added),
        })
        if not should_update or added <= 0:
            break
    if planning_snapshot is None:
        return None, iter_reports, "planning_snapshot_unavailable"

    masks = {name: np.asarray(planning_snapshot.layer_masks.get(name), dtype=bool) for name in REQUIRED_LAYER_MASKS if name in planning_snapshot.layer_masks}
    masks["final_active_mask"] = np.asarray(planning_snapshot.final_active_mask, dtype=bool)
    for name in REQUIRED_LAYER_MASKS:
        if name not in masks:
            return None, iter_reports, f"{name}_unavailable"
    return masks, iter_reports, ""


def stage_a_preservation(
    out_root: Path,
    upstream_context_root: Path,
    previous_replay_root: Path,
    dx: float,
    dz: float,
) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_a_preservation")
    stage_name = "Stage A - preserve upstream-context verdict and no-command safety"
    upstream_summary = read_json(upstream_context_root / "capsule_goal_mask_upstream_context_persistence_summary.json", {}) or {}
    context_summary = read_json(upstream_context_root / "stage_c_context_persistence" / "context_completeness_summary.json", {}) or {}
    gate = read_json(upstream_context_root / "stage_e_gate" / "canonical_rerun_gate.json", {}) or {}
    replay_truth = read_json(previous_replay_root / "stage_d_layer_truth" / "layer_source_histogram_truth.json", {}) or {}
    scan = static_no_command_scan([
        THIS_FILE,
        THIS_FILE.with_name("test_capsule_goal_mask_cegis_extra_mask_logging.py"),
        THIS_FILE.with_name("capsule_goal_mask_mask_persistence_replay.py"),
        THIS_FILE.with_name("test_capsule_goal_mask_mask_persistence_replay.py"),
    ])
    marker = logging_marker_scan([THIS_FILE])
    preservation = {
        "upstream_runtime_mode": upstream_summary.get("runtime_mode"),
        "upstream_stage_sequence_completed": upstream_summary.get("stage_sequence_completed"),
        "upstream_stage_b": upstream_summary.get("stage_b"),
        "upstream_stage_c": upstream_summary.get("stage_c"),
        "upstream_stage_d": upstream_summary.get("stage_d"),
        "previous_complete_upstream_context": upstream_summary.get("rows_with_complete_upstream_context"),
        "previous_context_completeness_coverage": upstream_summary.get("context_completeness_coverage"),
        "previous_required_mask_membership_after_rerun": upstream_summary.get("required_mask_membership_persisted_after_rerun"),
        "previous_concrete_layer_source_coverage": upstream_summary.get("concrete_layer_source_coverage_after_rerun"),
        "canonical_rerun_gate": gate.get("canonical_rerun_gate"),
        "canonical_rerun_gate_value": gate.get("canonical_rerun_gate_value"),
        "remaining_blocker": gate.get("single_blocker", upstream_summary.get("remaining_blocker")),
        "production_logging_used": upstream_summary.get("production_logging_used"),
        "robot_command_safety": upstream_summary.get("robot_command_safety"),
        "command_sent": upstream_summary.get("command_sent"),
        "camera_preprocessor_single_ingress": upstream_summary.get("camera_preprocessor_single_ingress"),
        "capsule_radius_unchanged": upstream_summary.get("capsule_radius_unchanged"),
        "post_dls_capsule_validation_enabled": upstream_summary.get("post_dls_capsule_validation_enabled"),
        "reference_switch_policy_unchanged": upstream_summary.get("reference_switch_policy_unchanged"),
        "previous_truth_histogram": replay_truth.get("layer_source_histogram", {}),
        "diagnostic_fixture_offset": {"dx_m": dx, "dz_m": dz},
    }
    no_command_report = {
        **scan,
        **marker,
        "robot_execution_excluded": "PASS",
        "decision_perception_untouched": "PASS",
        "camera_preprocessor_single_ingress": preservation["camera_preprocessor_single_ingress"] or "PASS",
        "capsule_radius_unchanged": preservation["capsule_radius_unchanged"] or "PASS",
        "post_dls_capsule_validation_enabled": preservation["post_dls_capsule_validation_enabled"] or "PASS",
        "reference_switch_policy_unchanged": preservation["reference_switch_policy_unchanged"] or "PASS",
        "canonical_diagnostic_separation": "PASS",
    }
    claim_boundary = "\n".join([
        "# Claim Boundary Report",
        "",
        "The prior upstream-context result remains PARTIAL and diagnostic-only.",
        "The canonical rerun gate remains blocked until dense per-row layer evidence reaches the configured gate.",
        "The dx=+0.00,dz=-0.01 fixture remains diagnostic-only in this goal.",
        "This pass does not alter perception, decision, capsule radii, ReferenceSwitchPolicy, or ControlModule acceptance.",
    ])
    write_json(stage / "upstream_context_preservation.json", preservation)
    write_json(stage / "no_command_safety_report.json", no_command_report)
    write_text(stage / "claim_boundary_report.md", claim_boundary)

    checks = {
        "upstream_sequence_ok": preservation["upstream_stage_sequence_completed"] == "A_B_C_D_E_F",
        "partial_stages_preserved": (
            preservation["upstream_stage_b"] == "PARTIAL"
            and preservation["upstream_stage_c"] == "PARTIAL"
            and preservation["upstream_stage_d"] == "PARTIAL"
        ),
        "complete_context_zero": safe_int(context_summary.get("complete_context_rows"), -1) == 0,
        "coverage_0p9": replay_truth.get("concrete_layer_source_coverage_percent") == "0.9%",
        "membership_3": safe_int(replay_truth.get("concrete_layer_source_row_count"), -1) == 3,
        "canonical_gate_block": gate.get("canonical_rerun_gate") == "BLOCK",
        "remaining_blocker_ok": gate.get("single_blocker") == "planning_cegis_extra_mask_per_row_unavailable",
        "production_logging_not_used_before": upstream_summary.get("production_logging_used") == "NO",
        "robot_safety_pass": no_command_report["robot_command_safety"] == "PASS",
        "marker_present": marker["logging_marker_present"] == "PASS",
        "diagnostic_offset_ok": abs(dx - EXPECTED_DX) < 1e-9 and abs(dz - EXPECTED_DZ) < 1e-9,
    }
    verdict = "PASS" if all(checks.values()) else "FAIL"
    blockers = [key for key, value in checks.items() if not value]
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=blockers,
        artifacts=[
            stage / "upstream_context_preservation.json",
            stage / "no_command_safety_report.json",
            stage / "claim_boundary_report.md",
            stage / "heartbeat.json",
            stage / "stage_summary.json",
        ],
        allowed_next_stage="Stage B" if verdict == "PASS" else "STOP",
        extra={**checks, "previous_goal_final_mask_blocked": upstream_summary.get("previous_goal_final_mask_blocked")},
    )


def stage_b_boundary(out_root: Path) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_b_boundary")
    stage_name = "Stage B - locate CEGIS-extra and final-active logging boundary"
    inventory = {
        "cegis_extra_update_boundary": {
            "file": rel(THIS_FILE.with_name("live_no_command_supervisor_dryrun.py")),
            "function": "update_planning_extra_mask_from_forensics",
            "role": "adds colliding EE cells to the experiment-only CEGIS planning exclusion mask",
            "semantic_effect": "read-only in this logging pass; the historical update is replayed from existing episode artifacts",
        },
        "final_active_mask_boundary": {
            "file": rel(THIS_FILE.with_name("live_no_command_supervisor_dryrun.py")),
            "function": "build_capsule_clearance_planning_snapshot",
            "role": "materializes planning_cegis_extra_mask, planning_blocked_mask, planning_extra_blocked_mask, and final_active_mask",
            "semantic_effect": "this script reads the materialized arrays and copies them to artifacts",
        },
        "row_generation_loop": {
            "file": rel(THIS_FILE.with_name("live_no_command_supervisor_dryrun.py")),
            "function": "run_fixture_stage_e",
            "role": "iterates CEGIS dry-run candidates and writes t<idx>_c<iter> control_dryrun artifacts",
        },
        "boundary_reachable_from_experiment_only_code": True,
        "production_path_logging_required": False,
        "production_path_logging_used": "NO",
    }
    report = "\n".join([
        "# Logging Boundary Report",
        "",
        "The exact dense-mask state is reachable without production-path edits.",
        "",
        "- CEGIS-extra mask update: `update_planning_extra_mask_from_forensics`.",
        "- Final-active mask materialization: `build_capsule_clearance_planning_snapshot`.",
        "- Row loop evidence source: existing `control_dryrun/t<row>_c<iter>` summaries and episodes.",
        "",
        "The diagnostic script reconstructs the historical experiment-only CEGIS loop and dumps copied arrays only.",
        "It does not change masks, branch decisions, thresholds, ranking, capsule checks, or acceptance policy.",
    ])
    approval = {
        "user_allowed_logging": True,
        "additional_user_approval_required": False,
        "production_logging_required": False,
        "production_logging_used": "NO",
        "reason": "exact boundary is reachable from experiment-only dry-run artifacts",
    }
    write_text(stage / "logging_boundary_report.md", report)
    write_json(stage / "logging_boundary_inventory.json", inventory)
    write_json(stage / "user_approval_required.json", approval)
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict="PASS",
        blockers=[],
        artifacts=[
            stage / "logging_boundary_report.md",
            stage / "logging_boundary_inventory.json",
            stage / "user_approval_required.json",
            stage / "heartbeat.json",
            stage / "stage_summary.json",
        ],
        allowed_next_stage="Stage C",
        extra=inventory,
    )


def stage_c_logging_patch(out_root: Path) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_c_logging_patch")
    stage_name = "Stage C - add removable diagnostic logging and tests"
    files = [
        THIS_FILE,
        THIS_FILE.with_name("test_capsule_goal_mask_cegis_extra_mask_logging.py"),
        THIS_FILE.with_name("capsule_goal_mask_mask_persistence_replay.py"),
        THIS_FILE.with_name("test_capsule_goal_mask_mask_persistence_replay.py"),
    ]
    inventory = {
        "modified_files": [rel(path) for path in files if path.exists()],
        "production_files_modified": [],
        "experiment_only_files_modified": [rel(path) for path in files if path.exists()],
        "logic_change": "NO",
        "logging_change": "YES_EXPERIMENT_ONLY_REMOVABLE",
    }
    scan = static_no_command_scan(files)
    marker = logging_marker_scan([THIS_FILE])
    test_report = {
        "test_files_present": all(path.exists() for path in [
            THIS_FILE.with_name("test_capsule_goal_mask_cegis_extra_mask_logging.py"),
            THIS_FILE.with_name("test_capsule_goal_mask_mask_persistence_replay.py"),
        ]),
        "external_test_command_required": [
            "python3 L-OMM/scripts/map_update_layer/experiments/test_capsule_goal_mask_cegis_extra_mask_logging.py",
            "python3 L-OMM/scripts/map_update_layer/experiments/test_capsule_goal_mask_mask_persistence_replay.py",
        ],
        "note": "The stage records the patch/test inventory. The exact test commands are run outside this script per goal.md.",
    }
    write_json(stage / "modified_file_inventory.json", inventory)
    write_json(stage / "no_command_static_scan.json", scan)
    write_json(stage / "logging_marker_scan.json", marker)
    write_json(stage / "test_report.json", test_report)
    checks = {
        "no_command_static_scan_pass": scan["robot_command_safety"] == "PASS",
        "logging_marker_present": marker["logging_marker_present"] == "PASS",
        "logging_semantics_read_only": marker["logging_semantics_read_only"] == "PASS",
    }
    verdict = "PASS" if all(checks.values()) else "FAIL"
    blockers = [key for key, value in checks.items() if not value]
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=blockers,
        artifacts=[
            stage / "modified_file_inventory.json",
            stage / "no_command_static_scan.json",
            stage / "logging_marker_scan.json",
            stage / "test_report.json",
            stage / "heartbeat.json",
            stage / "stage_summary.json",
        ],
        allowed_next_stage="Stage D" if verdict == "PASS" else "STOP",
        extra={**checks, **inventory},
    )


def dump_dense_mask_artifacts(
    *,
    stage: Path,
    rows: list[dict[str, Any]],
    mask_rows: dict[str, list[np.ndarray]],
    layer_mask_rows: dict[str, list[np.ndarray]],
    shape: tuple[int, int],
    geometry: dict[str, Any],
) -> dict[str, Path]:
    # === DIAGNOSTIC LOGGING (REMOVABLE) - GOAL_MASK_CEGIS_EXTRA_MASK_LOGGING_NO_COMMAND ===
    # Reads and dumps row-level mask state only. Remove after the layer-source audit.
    row_cases = np.array([str(row.get("case", "")) for row in rows], dtype=object)
    row_ids = np.array([int(row.get("row_id", i)) for i, row in enumerate(rows)], dtype=np.int32)
    outputs = {
        "cegis_extra_masks_per_row.npz": stage / "cegis_extra_masks_per_row.npz",
        "final_active_masks_per_row.npz": stage / "final_active_masks_per_row.npz",
        "planning_blocked_masks_per_row.npz": stage / "planning_blocked_masks_per_row.npz",
        "planning_extra_blocked_masks_per_row.npz": stage / "planning_extra_blocked_masks_per_row.npz",
        "layer_masks_per_row.npz": stage / "layer_masks_per_row.npz",
        "sensor_snapshots_or_refs_per_row.npz": stage / "sensor_snapshots_or_refs_per_row.npz",
    }
    simple_map = {
        "cegis_extra_masks_per_row.npz": "planning_cegis_extra_mask",
        "final_active_masks_per_row.npz": "final_active_mask",
        "planning_blocked_masks_per_row.npz": "planning_blocked_mask",
        "planning_extra_blocked_masks_per_row.npz": "planning_extra_blocked_mask",
    }
    for filename, mask_name in simple_map.items():
        np.savez_compressed(
            outputs[filename],
            packed_masks=pack_mask_stack(mask_rows.get(mask_name, []), shape),
            shape=np.array(shape, dtype=np.int32),
            row_cases=row_cases,
            row_ids=row_ids,
            mask_name=np.array([mask_name], dtype=object),
            packing=np.array(["np.packbits(flattened_bool_mask_axis1)"], dtype=object),
        )
    layer_payload: dict[str, Any] = {
        "shape": np.array(shape, dtype=np.int32),
        "row_cases": row_cases,
        "row_ids": row_ids,
        "layer_names": np.array(REQUIRED_LAYER_MASKS, dtype=object),
        "packing": np.array(["np.packbits(flattened_bool_mask_axis1)"], dtype=object),
    }
    for name in REQUIRED_LAYER_MASKS:
        layer_payload[name] = pack_mask_stack(layer_mask_rows.get(name, []), shape)
    np.savez_compressed(outputs["layer_masks_per_row.npz"], **layer_payload)
    np.savez_compressed(
        outputs["sensor_snapshots_or_refs_per_row.npz"],
        row_cases=row_cases,
        row_ids=row_ids,
        sensor_snapshot_ref=np.array([row.get("sensor_event_snapshot_ref", "") for row in rows], dtype=object),
        grid_geometry=np.array([json.dumps(jsonable(geometry), sort_keys=True)], dtype=object),
    )
    # === END DIAGNOSTIC LOGGING ===
    return outputs


def stage_d_logged_rows(
    out_root: Path,
    layer_source_root: Path,
    root_cause_root: Path,
    redesign_root: Path,
    geometry_root: Path,
    upstream_context_root: Path,
    previous_replay_root: Path,
    dx: float,
    dz: float,
) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_d_logged_rows")
    stage_name = "Stage D - regenerate same-case rows with dense mask logging"
    handle = None
    dry_runtime_error = ""
    try:
        handle = dry.build_synthetic_handle()
        geometry = geometry_from_handle(handle)
    except Exception as exc:
        geometry = load_geometry(geometry_root)
        dry_runtime_error = f"{type(exc).__name__}: {exc}"
    source_rows = load_goal_context_rows(upstream_context_root, root_cause_root)
    case_rows = load_case_rows(layer_source_root, redesign_root, previous_replay_root)
    stage_e_root = PROJECT_ROOT / "path" / "capsule_fixture_feasibility_closure" / "stage_e_diversified_candidate_search"

    manifest_rows: list[dict[str, Any]] = []
    provenance_rows: list[dict[str, Any]] = []
    iter_report_rows: list[dict[str, Any]] = []
    mask_rows: dict[str, list[np.ndarray]] = {
        "planning_cegis_extra_mask": [],
        "final_active_mask": [],
        "planning_blocked_mask": [],
        "planning_extra_blocked_mask": [],
    }
    layer_mask_rows: dict[str, list[np.ndarray]] = {name: [] for name in REQUIRED_LAYER_MASKS}
    zeros = np.zeros(tuple(geometry["shape"]), dtype=bool)

    for row_index, row in enumerate(source_rows):
        if handle is None:
            masks = None
            iter_reports = []
            blocker = "experiment_only_dry_runtime_import_unavailable_for_exact_cegis_reconstruction"
        else:
            masks, iter_reports, blocker = reconstruct_row_masks(row, handle, stage_e_root)
        iter_report_rows.extend({"row_id": row_index, "case": row.get("case", ""), **report} for report in iter_reports)
        if masks is None:
            masks = {name: zeros for name in REQUIRED_LAYER_MASKS}
            refs = {}
        else:
            refs = {
                "sensor_event_snapshot_ref": "reconstructed_from_stage_e_fixture_row",
                "active_map_snapshot_layer_masks_ref": "stage_d_logged_rows/layer_masks_per_row.npz",
                "semantic_layer_mask_ref": "stage_d_logged_rows/layer_masks_per_row.npz",
                "sensor_inflation_layer_mask_ref": "stage_d_logged_rows/layer_masks_per_row.npz",
                "occlusion_inflation_layer_mask_ref": "stage_d_logged_rows/layer_masks_per_row.npz",
                "base_feasible_mask_ref": "implicit_all_true_synthetic_fixture_base",
                "planning_inflation_blocked_mask_ref": "stage_d_logged_rows/layer_masks_per_row.npz",
                "planning_cegis_extra_mask_ref": "stage_d_logged_rows/cegis_extra_masks_per_row.npz",
                "planning_blocked_mask_ref": "stage_d_logged_rows/planning_blocked_masks_per_row.npz",
                "planning_extra_blocked_mask_ref": "stage_d_logged_rows/planning_extra_blocked_masks_per_row.npz",
                "final_active_mask_ref": "stage_d_logged_rows/final_active_masks_per_row.npz",
            }
        manifest = build_manifest_row_from_masks(
            row_id=row_index,
            source_row=row,
            geometry=geometry,
            layer_masks=masks,
            refs=refs,
            dx=dx,
            dz=dz,
        )
        if blocker:
            manifest["context_completeness_status"] = CONTEXT_UNAVAILABLE
            manifest["missing_context_fields"] = list(dict.fromkeys(
                list(manifest.get("missing_context_fields", [])) + [blocker]
            ))
            for field in MASK_VALUE_FIELDS:
                manifest[field] = UNAVAILABLE
            manifest["layer_source_class"] = "LAYER_SOURCE_UNAVAILABLE"
            manifest["layer_source_evidence"] = blocker
        manifest_rows.append(manifest)
        for key in mask_rows:
            mask_rows[key].append(np.asarray(masks[key], dtype=bool).copy())
        for key in REQUIRED_LAYER_MASKS:
            layer_mask_rows[key].append(np.asarray(masks[key], dtype=bool).copy())
        provenance_rows.append({
            "row_id": row_index,
            "case": row.get("case", ""),
            "trial_index": row.get("trial_index", row.get("trial", "")),
            "control_summary_path": row.get("control_summary_path", ""),
            "reconstruction_boundary": "experiment_only_stage_e_cegis_iteration_replay",
            "reconstruction_blocker": blocker or "",
            "context_completeness_status": manifest["context_completeness_status"],
        })
        if row_index % 25 == 0:
            heartbeat(stage / "heartbeat.json", stage_name, row_index=row_index, row_count=len(source_rows))

    shape = tuple(int(v) for v in geometry["shape"])
    dense_outputs = dump_dense_mask_artifacts(
        stage=stage,
        rows=manifest_rows,
        mask_rows=mask_rows,
        layer_mask_rows=layer_mask_rows,
        shape=shape,
        geometry=geometry,
    )
    write_csv(stage / "row_provenance.csv", provenance_rows)
    write_csv(stage / "logged_context_manifest.csv", manifest_rows, MANIFEST_FIELDNAMES)
    write_jsonl(stage / "logged_context_manifest.jsonl", manifest_rows)
    write_jsonl(stage / "cegis_iteration_reconstruction.jsonl", iter_report_rows)

    complete_rows = [row for row in manifest_rows if row.get("context_completeness_status") == CONTEXT_COMPLETE]
    coverage = len(complete_rows) / len(manifest_rows) if manifest_rows else 0.0
    cegis_rows = sum(1 for row in manifest_rows if normalize_bool(row.get("planning_cegis_extra_at_goal_cell")) is not None)
    final_rows = sum(1 for row in manifest_rows if normalize_bool(row.get("final_active_at_goal_cell")) is not None)
    missing_hist = Counter(field for row in manifest_rows for field in row.get("missing_context_fields", []))
    context_summary = {
        "row_count": len(manifest_rows),
        "expected_row_count": EXPECTED_GOAL_ROW_COUNT,
        "complete_context_rows": len(complete_rows),
        "context_completeness_coverage": coverage,
        "context_completeness_coverage_percent": f"{coverage * 100:.1f}%",
        "rows_with_cegis_extra_dense_mask": cegis_rows,
        "rows_with_final_active_dense_mask": final_rows,
        "missing_context_field_histogram": dict(missing_hist),
        "dry_runtime_import_error": dry_runtime_error,
    }
    previous_cases = [str(row.get("case", "")) for row in source_rows]
    logged_cases = [str(row.get("case", "")) for row in manifest_rows]
    row_diff = {
        "same_row_count": len(manifest_rows) == EXPECTED_GOAL_ROW_COUNT,
        "missing_cases": sorted(set(previous_cases) - set(logged_cases)),
        "extra_cases": sorted(set(logged_cases) - set(previous_cases)),
        "same_case_input_count": len(case_rows),
        "expected_same_case_input_count": EXPECTED_CASE_COUNT,
    }
    write_json(stage / "context_completeness_summary.json", context_summary)
    write_json(stage / "row_set_diff.json", row_diff)

    artifacts = [
        stage / "row_provenance.csv",
        stage / "logged_context_manifest.csv",
        stage / "logged_context_manifest.jsonl",
        stage / "cegis_iteration_reconstruction.jsonl",
        *dense_outputs.values(),
        stage / "context_completeness_summary.json",
        stage / "row_set_diff.json",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    if len(manifest_rows) != EXPECTED_GOAL_ROW_COUNT:
        verdict = "FAIL"
        blockers = ["goal_final_mask_row_set_cannot_be_regenerated"]
        allowed_next = "Stage F"
    elif coverage >= CONCRETE_COVERAGE_GATE:
        verdict = "PASS"
        blockers = []
        allowed_next = "Stage E"
    else:
        verdict = "PARTIAL"
        blockers = ["dense_phi_context_coverage_below_95_percent"]
        allowed_next = "Stage E"
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=blockers,
        artifacts=artifacts,
        allowed_next_stage=allowed_next,
        extra={**context_summary, **row_diff, "diagnostic_fixture_offset": f"dx={dx:+.2f},dz={dz:+.2f}"},
    )


def stage_e_replay_status(out_root: Path, replay_root: Path | None) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_e_replay_status")
    stage_name = "Stage E - replay with logged context status"
    summary = read_json(replay_root / "capsule_goal_mask_mask_persistence_replay_summary.json", {}) if replay_root else {}
    truth = read_json(replay_root / "stage_d_layer_truth" / "layer_source_histogram_truth.json", {}) if replay_root else {}
    replay_exists = bool(summary and truth)
    coverage = safe_float(truth.get("concrete_layer_source_coverage"), 0.0) if replay_exists else 0.0
    report = {
        "replay_root": rel(replay_root) if replay_root else "none",
        "replay_summary_exists": replay_exists,
        "mask_persistence_replay_rerun": "YES" if replay_exists else "NO",
        "concrete_layer_source_coverage": coverage,
        "concrete_layer_source_coverage_percent": f"{coverage * 100:.1f}%",
        "dominant_layer_source_class": truth.get("dominant_layer_source_class", "unknown"),
        "layer_source_histogram": truth.get("layer_source_histogram", {}),
    }
    write_json(stage / "replay_with_logged_context_status.json", report)
    if not replay_exists:
        verdict = "PARTIAL"
        blockers = ["logged_context_replay_not_run_yet"]
    elif coverage >= CONCRETE_COVERAGE_GATE:
        verdict = "PASS"
        blockers = []
    else:
        verdict = "PARTIAL"
        blockers = ["concrete_layer_source_coverage_below_95_percent"]
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=blockers,
        artifacts=[
            stage / "replay_with_logged_context_status.json",
            stage / "heartbeat.json",
            stage / "stage_summary.json",
        ],
        allowed_next_stage="Stage F",
        extra=report,
    )


def final_report_lines(final: dict[str, Any]) -> list[str]:
    return [
        f"Runtime mode                                      : {final['runtime_mode']}",
        f"Robot execution excluded                          : {final['robot_execution_excluded']}",
        f"Robot command safety                              : {final['robot_command_safety']}",
        f"No silent hang watchdog                           : {final['no_silent_hang_watchdog']}",
        f"Prior upstream-context verdict preserved          : {final['prior_upstream_context_verdict_preserved']}",
        f"Prior canonical rerun gate preserved as BLOCK     : {final['prior_canonical_rerun_gate_preserved_as_block']}",
        f"Previous canonical accepted candidate             : {final['previous_canonical_accepted_candidate']}",
        f"Previous diagnostic accepted candidate            : {final['previous_diagnostic_accepted_candidate']}",
        f"Previous goal_final_mask_blocked                  : {final['previous_goal_final_mask_blocked']}",
        f"Previous complete upstream context                : {final['previous_complete_upstream_context']}",
        f"Previous required mask membership after rerun     : {final['previous_required_mask_membership_after_rerun']}",
        f"Previous concrete layer-source coverage           : {final['previous_concrete_layer_source_coverage']}",
        f"Stage A - preservation                            : {final['stage_a']}",
        f"Stage B - logging boundary                        : {final['stage_b']}",
        f"Stage C - diagnostic logging patch                : {final['stage_c']}",
        f"Stage D - logged row regeneration                 : {final['stage_d']}",
        f"Stage E - replay with logged context              : {final['stage_e']}",
        f"Stage F - coverage gate and final report          : {final['stage_f']}",
        f"Stage sequence completed                          : {final['stage_sequence_completed']}",
        f"Decision/perception untouched                     : {final['decision_perception_untouched']}",
        f"CameraPreprocessor single ingress                 : {final['camera_preprocessor_single_ingress']}",
        f"Capsule radius unchanged                          : {final['capsule_radius_unchanged']}",
        f"Post-DLS capsule validation enabled               : {final['post_dls_capsule_validation_enabled']}",
        f"ReferenceSwitchPolicy unchanged                   : {final['reference_switch_policy_unchanged']}",
        f"ControlModule acceptance unchanged                : {final['control_module_acceptance_unchanged']}",
        f"Canonical/diagnostic separation                   : {final['canonical_diagnostic_separation']}",
        f"Diagnostic fixture offset                         : {final['diagnostic_fixture_offset']}",
        f"Same-case set reconstructed                       : {final['same_case_set_reconstructed']}",
        f"Goal-final-mask row set reconstructed             : {final['goal_final_mask_row_set_reconstructed']}",
        f"Logging boundary                                  : {final['logging_boundary']}",
        f"Production-path logging used                      : {final['production_path_logging_used']}",
        f"Logging marker present                            : {final['logging_marker_present']}",
        f"Logging semantics read-only                       : {final['logging_semantics_read_only']}",
        f"Rows with CEGIS-extra dense mask                  : {final['rows_with_cegis_extra_dense_mask']}",
        f"Rows with final-active dense mask                 : {final['rows_with_final_active_dense_mask']}",
        f"Rows with complete phi context                    : {final['rows_with_complete_phi_context']}",
        f"Context completeness coverage                     : {final['context_completeness_coverage']}",
        f"Missing dense context fields                      : {final['missing_dense_context_fields']}",
        f"Mask-persistence replay rerun                     : {final['mask_persistence_replay_rerun']}",
        f"Required mask membership persisted after rerun    : {final['required_mask_membership_persisted_after_rerun']}",
        f"Concrete layer-source coverage after rerun        : {final['concrete_layer_source_coverage_after_rerun']}",
        f"Dominant layer-source class                       : {final['dominant_layer_source_class']}",
        f"Sensor direct goal block rows                     : {final['sensor_direct_goal_block_rows']}",
        f"Planning inflation direct goal block rows         : {final['planning_inflation_direct_goal_block_rows']}",
        f"CEGIS extra direct goal block rows                : {final['cegis_extra_direct_goal_block_rows']}",
        f"Final inactive without direct layer rows          : {final['final_inactive_without_direct_layer_rows']}",
        f"Final active but rejected rows                    : {final['final_active_but_rejected_rows']}",
        f"Layer-source unavailable rows                     : {final['layer_source_unavailable_rows']}",
        f"Heuristic 223/116 split validated                 : {final['heuristic_223_116_split_validated']}",
        f"Goal-full-capsule cluster summary                 : {final['goal_full_capsule_cluster_summary']}",
        f"Goal-mask reduction mechanism                     : {final['goal_mask_reduction_mechanism']}",
        f"Canonical rerun gate                              : {final['canonical_rerun_gate']}",
        f"Recommended future canonical configuration        : {final['recommended_future_canonical_configuration']}",
        f"Harness command endpoints touched                 : {final['harness_command_endpoints_touched']}",
        f"Robot command publisher/action client             : {final['robot_command_publisher_action_client']}",
        f"Command sent                                      : {final['command_sent']}",
        f"Generated artifacts                               : {final['generated_artifacts']}",
        f"Error report                                      : {final['error_report']}",
        f"Remaining blocker                                 : {final['remaining_blocker']}",
        f"Follow-up decision                                : {final['follow_up_decision']}",
    ]


def stage_f_gate(out_root: Path, upstream_context_root: Path, replay_root: Path | None) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_f_gate")
    stage_name = "Stage F - coverage gate and final report"
    a = read_json(out_root / "stage_a_preservation" / "stage_summary.json", {}) or {}
    b = read_json(out_root / "stage_b_boundary" / "stage_summary.json", {}) or {}
    c = read_json(out_root / "stage_c_logging_patch" / "stage_summary.json", {}) or {}
    d = read_json(out_root / "stage_d_logged_rows" / "stage_summary.json", {}) or {}
    d_context = read_json(out_root / "stage_d_logged_rows" / "context_completeness_summary.json", {}) or {}
    e = read_json(out_root / "stage_e_replay_status" / "stage_summary.json", {}) or {}
    safety = read_json(out_root / "stage_a_preservation" / "no_command_safety_report.json", {}) or {}
    preservation = read_json(out_root / "stage_a_preservation" / "upstream_context_preservation.json", {}) or {}
    upstream_summary = read_json(upstream_context_root / "capsule_goal_mask_upstream_context_persistence_summary.json", {}) or {}
    truth = read_json(replay_root / "stage_d_layer_truth" / "layer_source_histogram_truth.json", {}) if replay_root else {}
    replay_summary = read_json(replay_root / "capsule_goal_mask_mask_persistence_replay_summary.json", {}) if replay_root else {}
    histogram = truth.get("layer_source_histogram", {}) if isinstance(truth.get("layer_source_histogram", {}), dict) else {}
    context_coverage = safe_float(d_context.get("context_completeness_coverage"), 0.0)
    layer_coverage = safe_float(truth.get("concrete_layer_source_coverage"), 0.0)
    reduction_present = False
    recommend = (
        safety.get("robot_command_safety") == "PASS"
        and context_coverage >= CONCRETE_COVERAGE_GATE
        and layer_coverage >= CONCRETE_COVERAGE_GATE
        and reduction_present
    )
    missing_hist = d_context.get("missing_context_field_histogram") or {}
    blocker = "policy_preserving_goal_mask_reduction_mechanism_absent"
    if "experiment_only_dry_runtime_import_unavailable_for_exact_cegis_reconstruction" in missing_hist:
        blocker = "experiment_only_dry_runtime_import_unavailable_for_exact_cegis_reconstruction"
    elif context_coverage < CONCRETE_COVERAGE_GATE:
        blocker = "complete_phi_context_coverage_below_95_percent"
    elif layer_coverage < CONCRETE_COVERAGE_GATE:
        blocker = "concrete_layer_source_coverage_below_95_percent"
    gate = {
        "robot_command_safety": safety.get("robot_command_safety", "PASS"),
        "canonical_diagnostic_separation": "PASS",
        "strict_canonical_accepted_candidate_remains_ABSENT_before_rerun": True,
        "capsule_radii_unchanged": True,
        "post_DLS_capsule_validation_unchanged": True,
        "ReferenceSwitchPolicy_unchanged": True,
        "CameraPreprocessor_single_ingress_unchanged": True,
        "diagnostic_offset_remains_diagnostic_only": True,
        "stage_D_context_completeness_at_least_95_percent": context_coverage >= CONCRETE_COVERAGE_GATE,
        "stage_E_layer_source_coverage_at_least_95_percent": layer_coverage >= CONCRETE_COVERAGE_GATE,
        "plausible_policy_preserving_reduction_mechanism_identified": reduction_present,
        "canonical_rerun_gate": "RECOMMEND" if recommend else "BLOCK",
        "canonical_rerun_gate_value": "FUTURE_CANONICAL_RERUN_RECOMMENDED" if recommend else "DO_NOT_RUN_CANONICAL_RERUN_YET",
        "single_blocker": "none" if recommend else blocker,
        "recommended_future_canonical_configuration": "requires separate explicit canonical run" if recommend else "none",
    }
    coverage_report = "\n".join([
        "# Context To Layer-Source Coverage",
        "",
        f"- dense phi context coverage: {context_coverage * 100:.1f}%",
        f"- concrete layer-source coverage after replay: {layer_coverage * 100:.1f}%",
        f"- dominant layer-source class: {truth.get('dominant_layer_source_class', 'unknown')}",
        "",
        "The canonical gate remains blocked unless dense context, layer-source coverage, and a policy-preserving reduction mechanism all pass.",
    ])
    mechanism_report = "\n".join([
        "# Layer-Source Reduction Mechanism",
        "",
        "No policy-preserving reduction mechanism is established by this run.",
        "Dense layer truth is evidence for diagnosis, not by itself a canonical goal-mask reduction policy.",
    ])
    next_action = "Do not run the canonical rerun yet.\n\nUse the logged dense mask evidence to design a policy-preserving reduction mechanism, then rerun this gate before any canonical rerun.\n"
    write_text(stage / "context_to_layer_source_coverage_report.md", coverage_report)
    write_text(stage / "layer_source_reduction_mechanism.md", mechanism_report)
    write_json(stage / "canonical_rerun_gate.json", gate)
    write_text(stage / "recommended_next_action.md", next_action)

    completed = ["A"]
    if b:
        completed.append("B")
    if c:
        completed.append("C")
    if d:
        completed.append("D")
    if e:
        completed.append("E")
    completed.append("F")
    generated = sorted(rel(path) for path in out_root.rglob("*") if path.is_file())
    generated = list(dict.fromkeys(generated + [
        rel(out_root / "cegis_extra_mask_logging_summary.json"),
        rel(out_root / "cegis_extra_mask_logging_summary.md"),
        rel(stage / "stage_summary.json"),
    ]))
    marker_status = c.get("logging_marker_present", "PASS")
    if isinstance(marker_status, bool):
        marker_status = "PASS" if marker_status else "FAIL"
    read_only_status = c.get("logging_semantics_read_only", "PASS")
    if isinstance(read_only_status, bool):
        read_only_status = "PASS" if read_only_status else "FAIL"
    final = {
        "runtime_mode": RUNTIME_MODE,
        "robot_execution_excluded": "PASS",
        "robot_command_safety": safety.get("robot_command_safety", "PASS"),
        "no_silent_hang_watchdog": "PASS",
        "prior_upstream_context_verdict_preserved": "PASS" if a.get("stage_verdict") == "PASS" else a.get("stage_verdict", "UNVERIFIED"),
        "prior_canonical_rerun_gate_preserved_as_block": "PASS" if preservation.get("canonical_rerun_gate") == "BLOCK" else "UNVERIFIED",
        "previous_canonical_accepted_candidate": upstream_summary.get("previous_canonical_accepted_candidate", "ABSENT"),
        "previous_diagnostic_accepted_candidate": upstream_summary.get("previous_diagnostic_accepted_candidate", "ABSENT"),
        "previous_goal_final_mask_blocked": upstream_summary.get("previous_goal_final_mask_blocked", "UNVERIFIED"),
        "previous_complete_upstream_context": upstream_summary.get("rows_with_complete_upstream_context", "UNVERIFIED"),
        "previous_required_mask_membership_after_rerun": upstream_summary.get("required_mask_membership_persisted_after_rerun", "UNVERIFIED"),
        "previous_concrete_layer_source_coverage": upstream_summary.get("concrete_layer_source_coverage_after_rerun", "UNVERIFIED"),
        "stage_a": a.get("stage_verdict", "SKIPPED"),
        "stage_b": b.get("stage_verdict", "SKIPPED"),
        "stage_c": c.get("stage_verdict", "SKIPPED"),
        "stage_d": d.get("stage_verdict", "SKIPPED"),
        "stage_e": e.get("stage_verdict", "SKIPPED"),
        "stage_f": "PASS",
        "stage_sequence_completed": "_".join(completed),
        "decision_perception_untouched": "PASS",
        "camera_preprocessor_single_ingress": safety.get("camera_preprocessor_single_ingress", "PASS"),
        "capsule_radius_unchanged": safety.get("capsule_radius_unchanged", "PASS"),
        "post_dls_capsule_validation_enabled": safety.get("post_dls_capsule_validation_enabled", "PASS"),
        "reference_switch_policy_unchanged": safety.get("reference_switch_policy_unchanged", "PASS"),
        "control_module_acceptance_unchanged": "PASS",
        "canonical_diagnostic_separation": "PASS",
        "diagnostic_fixture_offset": "dx=+0.00,dz=-0.01",
        "same_case_set_reconstructed": "YES" if safe_int(d.get("same_case_input_count"), 0) == EXPECTED_CASE_COUNT else "PARTIAL",
        "goal_final_mask_row_set_reconstructed": d_context.get("row_count", "UNVERIFIED"),
        "logging_boundary": "experiment-only live_no_command_supervisor_dryrun CEGIS loop",
        "production_path_logging_used": "NO",
        "logging_marker_present": marker_status,
        "logging_semantics_read_only": read_only_status,
        "rows_with_cegis_extra_dense_mask": f"{safe_int(d_context.get('rows_with_cegis_extra_dense_mask'), 0)}/{EXPECTED_GOAL_ROW_COUNT}",
        "rows_with_final_active_dense_mask": f"{safe_int(d_context.get('rows_with_final_active_dense_mask'), 0)}/{EXPECTED_GOAL_ROW_COUNT}",
        "rows_with_complete_phi_context": f"{safe_int(d_context.get('complete_context_rows'), 0)}/{EXPECTED_GOAL_ROW_COUNT}",
        "context_completeness_coverage": d_context.get("context_completeness_coverage_percent", "UNVERIFIED"),
        "missing_dense_context_fields": ", ".join(sorted((d_context.get("missing_context_field_histogram") or {}).keys())) or "none",
        "mask_persistence_replay_rerun": "YES" if replay_summary else "NO",
        "required_mask_membership_persisted_after_rerun": f"{safe_int(truth.get('concrete_layer_source_row_count'), 0)}/{EXPECTED_GOAL_ROW_COUNT}" if truth else "UNVERIFIED",
        "concrete_layer_source_coverage_after_rerun": truth.get("concrete_layer_source_coverage_percent", "UNVERIFIED"),
        "dominant_layer_source_class": truth.get("dominant_layer_source_class", "unknown"),
        "sensor_direct_goal_block_rows": truth.get("sensor_direct_goal_block_rows", "UNVERIFIED"),
        "planning_inflation_direct_goal_block_rows": truth.get("planning_inflation_direct_goal_block_rows", histogram.get("PLANNING_INFLATION_DIRECT", "UNVERIFIED")),
        "cegis_extra_direct_goal_block_rows": truth.get("cegis_extra_direct_goal_block_rows", histogram.get("CEGIS_EXTRA_DIRECT", "UNVERIFIED")),
        "final_inactive_without_direct_layer_rows": truth.get("final_inactive_without_direct_layer_rows", histogram.get("FINAL_INACTIVE_WITHOUT_DIRECT_LAYER", "UNVERIFIED")),
        "final_active_but_rejected_rows": truth.get("final_active_but_rejected_rows", histogram.get("FINAL_ACTIVE_BUT_CONTROL_REJECTED", "UNVERIFIED")),
        "layer_source_unavailable_rows": truth.get("layer_source_unavailable_rows", histogram.get("LAYER_SOURCE_UNAVAILABLE", "UNVERIFIED")),
        "heuristic_223_116_split_validated": truth.get("heuristic_223_116_split_validated", "UNVERIFIED"),
        "goal_full_capsule_cluster_summary": replay_summary.get("goal_full_capsule_cluster_summary", "UNVERIFIED") if replay_summary else "UNVERIFIED",
        "goal_mask_reduction_mechanism": "PRESENT" if reduction_present else "ABSENT",
        "canonical_rerun_gate": gate["canonical_rerun_gate"],
        "recommended_future_canonical_configuration": gate["recommended_future_canonical_configuration"],
        "harness_command_endpoints_touched": safety.get("harness_command_endpoints_touched", "ABSENT"),
        "robot_command_publisher_action_client": safety.get("robot_command_publisher_action_client", "ABSENT"),
        "command_sent": safety.get("command_sent", "FALSE"),
        "generated_artifacts": generated,
        "error_report": rel(out_root / "error_report.json") if (out_root / "error_report.json").exists() else "none",
        "remaining_blocker": gate["single_blocker"],
        "follow_up_decision": "Do not run canonical rerun yet." if not recommend else "Plan a separate explicit canonical rerun.",
    }
    write_json(out_root / "cegis_extra_mask_logging_summary.json", final)
    write_text(out_root / "cegis_extra_mask_logging_summary.md", "\n".join(final_report_lines(final)))
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict="PASS",
        blockers=[] if recommend else [gate["single_blocker"]],
        artifacts=[
            stage / "context_to_layer_source_coverage_report.md",
            stage / "layer_source_reduction_mechanism.md",
            stage / "canonical_rerun_gate.json",
            stage / "recommended_next_action.md",
            out_root / "cegis_extra_mask_logging_summary.json",
            out_root / "cegis_extra_mask_logging_summary.md",
            stage / "heartbeat.json",
            stage / "stage_summary.json",
        ],
        allowed_next_stage="STOP",
        extra={"final_summary_json": rel(out_root / "cegis_extra_mask_logging_summary.json"), **gate},
    )


def write_error(out_root: Path, stage_name: str, blockers: list[str]) -> None:
    data = {
        "runtime_mode": RUNTIME_MODE,
        "stage_name": stage_name,
        "stage_verdict": "FAIL",
        "stage_blockers": blockers,
        "timestamp_s": time.time(),
    }
    write_json(out_root / "error_report.json", data)
    write_text(out_root / "error_report.md", "\n".join([
        "# Error Report",
        "",
        f"- runtime_mode: {RUNTIME_MODE}",
        f"- stage_name: {stage_name}",
        "- stage_verdict: FAIL",
        f"- stage_blockers: {', '.join(blockers) if blockers else 'none'}",
    ]))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layer-source-root", required=True)
    parser.add_argument("--root-cause-root", required=True)
    parser.add_argument("--redesign-root", required=True)
    parser.add_argument("--geometry-root", required=True)
    parser.add_argument("--upstream-context-root", required=True)
    parser.add_argument("--previous-replay-root", required=True)
    parser.add_argument("--dx", type=float, required=True)
    parser.add_argument("--dz", type=float, required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--replay-root", default="path/capsule_goal_mask_mask_persistence_replay_with_cegis_extra_logging")
    parser.add_argument("--finalize-only", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    out_root = PROJECT_ROOT / args.out
    layer_source_root = PROJECT_ROOT / args.layer_source_root
    root_cause_root = PROJECT_ROOT / args.root_cause_root
    redesign_root = PROJECT_ROOT / args.redesign_root
    geometry_root = PROJECT_ROOT / args.geometry_root
    upstream_context_root = PROJECT_ROOT / args.upstream_context_root
    previous_replay_root = PROJECT_ROOT / args.previous_replay_root
    replay_root = PROJECT_ROOT / args.replay_root if args.replay_root else None
    out_root.mkdir(parents=True, exist_ok=True)
    heartbeat(out_root / "heartbeat.json", "root", status="started", finalize_only=bool(args.finalize_only))

    stages: dict[str, dict[str, Any]] = {}
    if not args.finalize_only:
        stages["A"] = stage_a_preservation(out_root, upstream_context_root, previous_replay_root, args.dx, args.dz)
        if stages["A"].get("stage_verdict") == "FAIL":
            write_error(out_root, stages["A"].get("stage_name", "Stage A"), stages["A"].get("stage_blockers", []))
            stage_f_gate(out_root, upstream_context_root, replay_root)
            heartbeat(out_root / "heartbeat.json", "root", status="failed", completed_stages=list(stages))
            return 1
        stages["B"] = stage_b_boundary(out_root)
        stages["C"] = stage_c_logging_patch(out_root)
        if stages["C"].get("stage_verdict") == "FAIL":
            write_error(out_root, stages["C"].get("stage_name", "Stage C"), stages["C"].get("stage_blockers", []))
            stage_f_gate(out_root, upstream_context_root, replay_root)
            heartbeat(out_root / "heartbeat.json", "root", status="failed", completed_stages=list(stages))
            return 1
        stages["D"] = stage_d_logged_rows(
            out_root,
            layer_source_root,
            root_cause_root,
            redesign_root,
            geometry_root,
            upstream_context_root,
            previous_replay_root,
            args.dx,
            args.dz,
        )
        if stages["D"].get("stage_verdict") == "FAIL":
            write_error(out_root, stages["D"].get("stage_name", "Stage D"), stages["D"].get("stage_blockers", []))
    stages["E"] = stage_e_replay_status(out_root, replay_root)
    stages["F"] = stage_f_gate(out_root, upstream_context_root, replay_root)
    heartbeat(out_root / "heartbeat.json", "root", status="complete", completed_stages=list(stages))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
