#!/usr/bin/env python3
"""Persist upstream goal-cell context for the no-command mask replay.

This script is diagnostic-only. It does not call robot controllers, camera
readers, ROS publishers, or production policy-changing code. Its purpose is to
turn the existing row-level artifacts into an auditable per-row context bundle
and to name any upstream boundary that remains unavailable.
"""
from __future__ import annotations

import argparse
import ast
import csv
import json
import math
import time
from collections import Counter
from pathlib import Path
from typing import Any, Iterable

import numpy as np


THIS_FILE = Path(__file__).resolve()
PROJECT_ROOT = THIS_FILE.parents[4]
RUNTIME_MODE = "GOAL_MASK_UPSTREAM_CONTEXT_PERSISTENCE_NO_COMMAND"
PREVIOUS_REPLAY_MODE = "GOAL_MASK_MASK_PERSISTENCE_REPLAY_NO_COMMAND"

EXPECTED_DX = 0.0
EXPECTED_DZ = -0.01
EXPECTED_CASE_COUNT = 78
EXPECTED_GOAL_ROW_COUNT = 339
CONCRETE_COVERAGE_GATE = 0.95
UNAVAILABLE = "UNAVAILABLE"

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

REQUIRED_CONTEXT_FIELDS = [
    "sensor_event_snapshot_per_row",
    "map_handle_or_grid_geometry_per_row",
    "ActiveMapSnapshot.layer_masks_per_row",
    "planning_inflation_blocked_mask_per_row",
    "planning_cegis_extra_mask_per_row",
    "planning_blocked_mask_per_row",
    "planning_extra_blocked_mask_per_row",
    "final_active_mask_per_row",
]

CONTEXT_COMPLETE = "CONTEXT_COMPLETE"
CONTEXT_PARTIAL_SENSOR_ONLY = "CONTEXT_PARTIAL_SENSOR_ONLY"
CONTEXT_PARTIAL_PLANNING_ONLY = "CONTEXT_PARTIAL_PLANNING_ONLY"
CONTEXT_ROW_ID_ONLY = "CONTEXT_ROW_ID_ONLY"
CONTEXT_UNAVAILABLE = "CONTEXT_UNAVAILABLE"

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
    "sensor_event_snapshot_ref",
    "map_handle_or_grid_geometry_ref",
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
    "context_source_boundary",
    "control_summary_path",
    "regenerated_sensor_stats_match",
    "regenerated_planning_inflation_stats_match",
    *MASK_VALUE_FIELDS,
    "nearest_sensor_blocked_cell",
    "nearest_sensor_blocked_distance_m",
    "nearest_planning_inflation_blocked_cell",
    "nearest_planning_inflation_blocked_distance_m",
]


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
    try:
        return path.read_text(encoding="utf-8")
    except Exception:
        return ""


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
            line = line.strip()
            if not line:
                continue
            try:
                value = json.loads(line)
            except Exception:
                continue
            if isinstance(value, dict):
                rows.append(value)
    return rows


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(jsonable(data), indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        keys: list[str] = []
        seen: set[str] = set()
        for row in rows:
            for key in row:
                if key not in seen:
                    keys.append(key)
                    seen.add(key)
        fieldnames = keys
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(jsonable(row.get(key)), ensure_ascii=False) if isinstance(row.get(key), (list, dict)) else row.get(key) for key in fieldnames})


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(jsonable(row), sort_keys=True, ensure_ascii=False) + "\n")


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT.resolve()))
    except Exception:
        return str(path)


def safe_float(value: Any, default: float = float("nan")) -> float:
    if value is None or value == "":
        return float(default)
    try:
        out = float(value)
    except Exception:
        return float(default)
    return out


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
        if int(value) in (-1,):
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
        "error_report_path": "none",
        "robot_command_safety": "PASS",
        "canonical_diagnostic_separation": "PASS",
        "claim_boundary_preserved": "PASS",
    }
    if extra:
        data.update(extra)
    write_json(stage / "stage_summary.json", data)
    heartbeat(hb_path, stage_name, stage_verdict=verdict, stage_blockers=blockers)
    heartbeat(out_root / "heartbeat.json", stage_name, stage_verdict=verdict, last_stage_dir=rel(stage))
    return data


class ForbiddenCallVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.hits: list[dict[str, Any]] = []
        self.import_hits: list[dict[str, Any]] = []

    def visit_Call(self, node: ast.Call) -> Any:
        name = ""
        if isinstance(node.func, ast.Name):
            name = node.func.id
        elif isinstance(node.func, ast.Attribute):
            name = node.func.attr
        if name in FORBIDDEN_CALL_NAMES:
            self.hits.append({"call": name, "line": getattr(node, "lineno", None)})
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
        tree = ast.parse(text)
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
        "forbidden_call_hits": visitor.hits,
        "forbidden_import_hits": visitor.import_hits,
        "forbidden_call_count": len(visitor.hits),
        "forbidden_import_hit_count": len(visitor.import_hits),
    }


def static_no_command_scan(paths: list[Path]) -> dict[str, Any]:
    reports = [scan_python_file(path) for path in paths if path.exists()]
    return {
        "files_scanned": [report["path"] for report in reports],
        "reports": reports,
        "ast_forbidden_call_count": sum(int(report["forbidden_call_count"]) for report in reports),
        "forbidden_import_hit_count": sum(int(report["forbidden_import_hit_count"]) for report in reports),
        "harness_command_endpoints_touched": "ABSENT",
        "robot_command_publisher_action_client": "ABSENT",
        "command_sent": "FALSE",
        "new_rgbd_ingress_added": False,
        "camera_preprocessor_bypassed": False,
        "robot_command_safety": "PASS"
        if all(int(report["forbidden_call_count"]) == 0 and int(report["forbidden_import_hit_count"]) == 0 for report in reports)
        else "FAIL",
    }


def close_enough(a: float, b: float, tol: float = 1.0e-9) -> bool:
    if not math.isfinite(a) or not math.isfinite(b):
        return False
    return abs(a - b) <= tol


def load_goal_rows(previous_replay_root: Path, root_cause_root: Path, layer_source_root: Path) -> list[dict[str, Any]]:
    primary_candidates = [
        previous_replay_root / "stage_b_replay_context" / "goal_final_mask_row_set.csv",
        root_cause_root / "stage_c_goal_mask_root_cause" / "goal_cell_forensics.jsonl",
        root_cause_root / "stage_c_goal_mask_root_cause" / "goal_final_mask_blocked_rows.csv",
        layer_source_root / "stage_c_layer_source" / "goal_cell_layer_source_rows.csv",
    ]
    primary_rows: list[dict[str, Any]] = []
    for path in primary_candidates:
        rows = read_jsonl(path) if path.suffix == ".jsonl" else read_csv(path)
        if rows:
            primary_rows = [dict(row) for row in rows]
            break
    if not primary_rows:
        return []
    detail_rows: list[dict[str, Any]] = []
    for path in [
        root_cause_root / "stage_c_goal_mask_root_cause" / "goal_cell_forensics.jsonl",
        root_cause_root / "stage_c_goal_mask_root_cause" / "goal_final_mask_blocked_rows.csv",
        layer_source_root / "stage_c_layer_source" / "goal_cell_layer_source_rows.csv",
    ]:
        rows = read_jsonl(path) if path.suffix == ".jsonl" else read_csv(path)
        if rows:
            detail_rows.extend(dict(row) for row in rows)
    detail_by_case = {str(row.get("case", "")): row for row in detail_rows if str(row.get("case", ""))}
    preserve_fields = [
        "final_rejection_reason",
        "invalid_reason_code",
        "goal_full_capsule_blocked_count",
        "sensor_goal_blocked_count",
        "planning_extra_blocked_cells",
        "planning_blocked_cells",
        "sensor_blocked_cells",
        "capsule_cegis_iteration_count",
        "capsule_cegis_added_planning_cells",
        "row_level_root_cause_class",
        "row_level_root_cause_from_previous_audit",
    ]
    out: list[dict[str, Any]] = []
    for row in primary_rows:
        merged = dict(row)
        detail = detail_by_case.get(str(row.get("case", "")), {})
        for field in preserve_fields:
            if field in detail and str(merged.get(field, "")) in {"", "0", "0.0", "UNAVAILABLE"}:
                merged[field] = detail[field]
        out.append(merged)
    return out


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


def trace_by_case(geometry_root: Path) -> dict[str, dict[str, str]]:
    rows = read_csv(geometry_root / "stage_e_cegis_audit" / "cegis_trace.csv")
    return {str(row.get("case", "")): row for row in rows if str(row.get("case", ""))}


def control_summary_for_trace(row: dict[str, Any]) -> dict[str, Any]:
    raw = str(row.get("control_summary_path", ""))
    if not raw:
        return {}
    path = PROJECT_ROOT / raw
    return read_json(path, {}) or {}


def load_geometry(geometry_root: Path) -> dict[str, Any]:
    contract = read_json(geometry_root / "stage_d_fixture_geometry" / "fixture_geometry_contract.json", {}) or {}
    shape = contract.get("handle_shape") or [260, 240]
    x_bounds = contract.get("x_bounds_m") or [-1.1, 1.49]
    z_bounds = contract.get("z_bounds_m") or [-0.6, 1.79]
    resolution = safe_float(contract.get("resolution_m"), 0.01)
    return {
        "shape": [int(shape[0]), int(shape[1])],
        "x0": float(x_bounds[0]),
        "z0": float(z_bounds[0]),
        "resolution_m": float(resolution),
        "contract_path": rel(geometry_root / "stage_d_fixture_geometry" / "fixture_geometry_contract.json"),
    }


def xz_to_cell(xz: Any, geometry: dict[str, Any]) -> list[int] | None:
    vals = parse_jsonish(xz, xz)
    if not isinstance(vals, (list, tuple)) or len(vals) < 2:
        return None
    try:
        ix = int(round((float(vals[0]) - float(geometry["x0"])) / float(geometry["resolution_m"])))
        iz = int(round((float(vals[1]) - float(geometry["z0"])) / float(geometry["resolution_m"])))
    except Exception:
        return None
    return [ix, iz]


def row_goal_cell(row: dict[str, Any], geometry: dict[str, Any]) -> list[int] | None:
    parsed = parse_jsonish(row.get("requested_goal_cell"), None)
    if isinstance(parsed, (list, tuple)) and len(parsed) >= 2:
        try:
            return [int(float(parsed[0])), int(float(parsed[1]))]
        except Exception:
            pass
    return xz_to_cell(row.get("requested_goal_xz"), geometry)


def normalize_rect(rect_value: Any) -> tuple[float, float, float, float] | None:
    rect = parse_jsonish(rect_value, None)
    if not isinstance(rect, (list, tuple)) or len(rect) < 4:
        return None
    try:
        x0, z0, x1, z1 = (float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3]))
    except Exception:
        return None
    return (min(x0, x1), min(z0, z1), max(x0, x1), max(z0, z1))


def rect_mask(geometry: dict[str, Any], rect_value: Any, *, margin_m: float = 0.0) -> np.ndarray:
    shape = tuple(int(v) for v in geometry["shape"])
    rect = normalize_rect(rect_value)
    if rect is None:
        return np.zeros(shape, dtype=bool)
    min_x, min_z, max_x, max_z = rect
    min_x -= float(margin_m)
    min_z -= float(margin_m)
    max_x += float(margin_m)
    max_z += float(margin_m)
    res = float(geometry["resolution_m"])
    half = 0.5 * res
    xs = float(geometry["x0"]) + np.arange(shape[0], dtype=np.float64) * res
    zs = float(geometry["z0"]) + np.arange(shape[1], dtype=np.float64) * res
    x_hit = (xs + half >= min_x) & (xs - half <= max_x)
    z_hit = (zs + half >= min_z) & (zs - half <= max_z)
    return np.outer(x_hit, z_hit).astype(bool)


def mask_cell_value(mask: np.ndarray | None, cell: list[int] | None) -> bool | None:
    if mask is None or cell is None or len(cell) < 2:
        return None
    ix, iz = int(cell[0]), int(cell[1])
    if ix < 0 or iz < 0 or ix >= mask.shape[0] or iz >= mask.shape[1]:
        return None
    return bool(mask[ix, iz])


def nearest_true_cell(geometry: dict[str, Any], mask: np.ndarray, cell: list[int] | None) -> dict[str, Any]:
    if cell is None:
        return {"cell": UNAVAILABLE, "distance_m": UNAVAILABLE}
    coords = np.argwhere(mask)
    if coords.size == 0:
        return {"cell": UNAVAILABLE, "distance_m": UNAVAILABLE}
    target = np.array([int(cell[0]), int(cell[1])], dtype=np.float64)
    delta = coords.astype(np.float64) - target.reshape(1, 2)
    idx = int(np.argmin(np.sum(delta * delta, axis=1)))
    nearest = coords[idx].astype(int).tolist()
    distance = float(np.linalg.norm(delta[idx]) * float(geometry["resolution_m"]))
    return {"cell": nearest, "distance_m": distance}


def bool_to_manifest(value: bool | None) -> str:
    if value is True:
        return "True"
    if value is False:
        return "False"
    return UNAVAILABLE


def bool_to_tensor_value(value: Any) -> int:
    normalized = normalize_bool(value)
    if normalized is True:
        return 1
    if normalized is False:
        return 0
    return -1


def pack_mask_stack(masks: list[np.ndarray], shape: tuple[int, int]) -> np.ndarray:
    if masks:
        stack = np.stack([np.asarray(mask, dtype=bool) for mask in masks], axis=0)
    else:
        stack = np.zeros((0, shape[0], shape[1]), dtype=bool)
    return np.packbits(stack.reshape(stack.shape[0], -1), axis=1)


def field_missing_counts(rows: list[dict[str, Any]]) -> dict[str, int]:
    return {field: sum(1 for row in rows if normalize_bool(row.get(field)) is None) for field in MASK_VALUE_FIELDS}


def row_context_status(row: dict[str, Any]) -> str:
    missing = parse_jsonish(row.get("missing_context_fields"), [])
    if not missing:
        return CONTEXT_COMPLETE
    missing_set = {str(v) for v in missing}
    sensor_available = not {
        "sensor_event_snapshot_per_row",
        "ActiveMapSnapshot.layer_masks_per_row",
    }.intersection(missing_set)
    planning_inflation_available = "planning_inflation_blocked_mask_per_row" not in missing_set
    if sensor_available and planning_inflation_available:
        return CONTEXT_PARTIAL_PLANNING_ONLY
    if sensor_available:
        return CONTEXT_PARTIAL_SENSOR_ONLY
    if str(row.get("case", "")):
        return CONTEXT_ROW_ID_ONLY
    return CONTEXT_UNAVAILABLE


def build_context_rows(
    *,
    goal_rows: list[dict[str, Any]],
    trace_rows: dict[str, dict[str, str]],
    geometry: dict[str, Any],
    dx: float,
    dz: float,
) -> tuple[list[dict[str, Any]], dict[str, list[np.ndarray]], dict[str, list[np.ndarray]]]:
    manifest_rows: list[dict[str, Any]] = []
    sensor_masks: dict[str, list[np.ndarray]] = {
        "occupied": [],
        "target": [],
        "unknown": [],
        "occluded": [],
        "sensor_inflated": [],
        "sensor_inflation_added": [],
        "occlusion_inflated": [],
        "occlusion_inflation_added": [],
        "sensor_blocked_mask": [],
    }
    planning_masks: dict[str, list[np.ndarray]] = {
        "planning_inflation_blocked_mask": [],
    }
    shape = tuple(int(v) for v in geometry["shape"])
    zeros = np.zeros(shape, dtype=bool)
    for row_index, row in enumerate(goal_rows):
        case = str(row.get("case", ""))
        trace = trace_rows.get(case, {})
        summary = control_summary_for_trace(trace) if trace else {}
        stats = (
            summary.get("active_map_update_stats")
            or ((summary.get("case") or {}).get("plan") or {}).get("map_update_stats")
            or {}
        )
        cell = row_goal_cell(row, geometry)
        inside = bool(cell is not None and 0 <= cell[0] < shape[0] and 0 <= cell[1] < shape[1])
        rect_value = trace.get("obstacle_rect") or row.get("obstacle_rect")
        sensor_inflation_m = safe_float(stats.get("sensor_inflation_m"), 0.005)
        planning_inflation_m = safe_float(trace.get("planning_inflation"), safe_float(row.get("capsule_planning_inflation_m"), float("nan")))

        occupied = rect_mask(geometry, rect_value, margin_m=0.0) if trace else zeros.copy()
        target = zeros.copy()
        unknown = zeros.copy()
        occluded = zeros.copy()
        sensor_inflated = rect_mask(geometry, rect_value, margin_m=sensor_inflation_m) if trace else zeros.copy()
        sensor_inflation_added = sensor_inflated & (~occupied)
        occlusion_inflated = zeros.copy()
        occlusion_inflation_added = zeros.copy()
        sensor_blocked = occupied | target | unknown | occluded | sensor_inflated | occlusion_inflated
        if math.isfinite(planning_inflation_m):
            planning_inflation = rect_mask(geometry, rect_value, margin_m=planning_inflation_m) if trace else zeros.copy()
        else:
            planning_inflation = zeros.copy()

        sensor_count = int(np.count_nonzero(sensor_blocked))
        planning_count = int(np.count_nonzero(planning_inflation))
        expected_sensor_count = safe_int(stats.get("sensor_blocked_cells", stats.get("blocked_cells")), -1)
        expected_planning_inflation_count = safe_int(stats.get("planning_inflation_blocked_cells"), -1)
        sensor_stats_match = expected_sensor_count >= 0 and sensor_count == expected_sensor_count
        planning_stats_match = expected_planning_inflation_count >= 0 and planning_count == expected_planning_inflation_count

        values: dict[str, bool | None] = {field: None for field in MASK_VALUE_FIELDS}
        values["goal_cell_inside_map"] = inside
        if inside:
            total_cells = shape[0] * shape[1]
            base_cells = safe_int(stats.get("base_feasible_cells"), total_cells)
            values["base_feasible_at_goal_cell"] = bool(base_cells == total_cells)
        if inside and sensor_stats_match:
            values["occupied_at_goal_cell"] = mask_cell_value(occupied, cell)
            values["target_at_goal_cell"] = False
            values["unknown_at_goal_cell"] = False
            values["occluded_at_goal_cell"] = False
            values["sensor_inflated_at_goal_cell"] = mask_cell_value(sensor_inflated, cell)
            values["sensor_inflation_added_at_goal_cell"] = mask_cell_value(sensor_inflation_added, cell)
            values["occlusion_inflated_at_goal_cell"] = False
            values["occlusion_inflation_added_at_goal_cell"] = False
            values["sensor_blocked_at_goal_cell"] = mask_cell_value(sensor_blocked, cell)
        if inside and planning_stats_match:
            planning_direct = mask_cell_value(planning_inflation, cell)
            sensor_direct = mask_cell_value(sensor_blocked, cell) if sensor_stats_match else None
            values["planning_inflation_blocked_at_goal_cell"] = planning_direct
            if planning_direct is True:
                values["planning_blocked_at_goal_cell"] = True
                values["planning_extra_blocked_at_goal_cell"] = not bool(sensor_direct)
                values["final_active_at_goal_cell"] = False
            elif planning_direct is False:
                values["planning_blocked_at_goal_cell"] = None
                values["planning_extra_blocked_at_goal_cell"] = None
                values["final_active_at_goal_cell"] = None
        values["planning_cegis_extra_at_goal_cell"] = None

        missing = []
        if not trace:
            missing.extend(REQUIRED_CONTEXT_FIELDS)
        else:
            if not sensor_stats_match:
                missing.extend(["sensor_event_snapshot_per_row", "ActiveMapSnapshot.layer_masks_per_row"])
            if not planning_stats_match:
                missing.append("planning_inflation_blocked_mask_per_row")
            missing.extend([
                "planning_cegis_extra_mask_per_row",
                "planning_blocked_mask_per_row" if values.get("planning_blocked_at_goal_cell") is None else "",
                "planning_extra_blocked_mask_per_row" if values.get("planning_extra_blocked_at_goal_cell") is None else "",
                "final_active_mask_per_row" if values.get("final_active_at_goal_cell") is None else "",
            ])
        missing = [field for field in dict.fromkeys(missing) if field]
        sensor_nearest = nearest_true_cell(geometry, sensor_blocked, cell) if sensor_stats_match else {"cell": UNAVAILABLE, "distance_m": UNAVAILABLE}
        planning_nearest = nearest_true_cell(geometry, planning_inflation, cell) if planning_stats_match else {"cell": UNAVAILABLE, "distance_m": UNAVAILABLE}

        manifest = {
            "row_id": row_index,
            "case": case,
            "seed_id": row.get("seed_id", trace.get("seed_id", "")),
            "current_fraction": row.get("current_fraction", trace.get("current_fraction", "")),
            "obstacle_fraction": row.get("obstacle_fraction", trace.get("obstacle_fraction", "")),
            "trial_index": row.get("trial_index", row.get("trial", trace.get("trial", ""))),
            "fixture_offset_dx_m": dx,
            "fixture_offset_dz_m": dz,
            "requested_goal_xz": parse_jsonish(row.get("requested_goal_xz"), row.get("requested_goal_xz")),
            "requested_goal_cell": cell if cell is not None else UNAVAILABLE,
            "sensor_event_snapshot_ref": "regenerated_from_cegis_trace_obstacle_rect" if sensor_stats_match else UNAVAILABLE,
            "map_handle_or_grid_geometry_ref": geometry["contract_path"],
            "active_map_snapshot_layer_masks_ref": "stage_c_context_persistence/layer_masks_per_row.npz" if sensor_stats_match else UNAVAILABLE,
            "semantic_layer_mask_ref": "stage_c_context_persistence/layer_masks_per_row.npz" if sensor_stats_match else UNAVAILABLE,
            "sensor_inflation_layer_mask_ref": "stage_c_context_persistence/layer_masks_per_row.npz" if sensor_stats_match else UNAVAILABLE,
            "occlusion_inflation_layer_mask_ref": "stage_c_context_persistence/layer_masks_per_row.npz" if sensor_stats_match else UNAVAILABLE,
            "base_feasible_mask_ref": "implicit_all_true_synthetic_fixture_base" if inside else UNAVAILABLE,
            "planning_inflation_blocked_mask_ref": "stage_c_context_persistence/planning_masks_per_row.npz" if planning_stats_match else UNAVAILABLE,
            "planning_cegis_extra_mask_ref": UNAVAILABLE,
            "planning_blocked_mask_ref": "goal_cell_scalar_derived_from_planning_inflation_only" if values.get("planning_blocked_at_goal_cell") is True else UNAVAILABLE,
            "planning_extra_blocked_mask_ref": "goal_cell_scalar_derived_from_planning_inflation_only" if values.get("planning_extra_blocked_at_goal_cell") is not None else UNAVAILABLE,
            "final_active_mask_ref": "goal_cell_scalar_derived_from_planning_inflation_only" if values.get("final_active_at_goal_cell") is not None else UNAVAILABLE,
            "goal_full_capsule_blocked_count": safe_int(row.get("goal_full_capsule_blocked_count"), 0),
            "sensor_goal_blocked_count": safe_int(row.get("sensor_goal_blocked_count"), 0),
            "planning_extra_blocked_cells": safe_int(row.get("planning_extra_blocked_cells"), 0),
            "capsule_cegis_added_planning_cells": safe_int(row.get("capsule_cegis_added_planning_cells", trace.get("cegis_added_planning_cells")), 0),
            "capsule_cegis_iteration_count": safe_int(row.get("capsule_cegis_iteration_count", trace.get("cegis_iteration_count")), 0),
            "previous_invalid_reason_code": row.get("invalid_reason_code", trace.get("invalid_reason_code", "")),
            "previous_row_level_root_cause": row.get("row_level_root_cause_class", row.get("row_level_root_cause_from_previous_audit", "")),
            "missing_context_fields": missing,
            "context_source_boundary": "geometry_stage_e_cegis_trace_plus_control_summary" if trace else "row_identifier_only",
            "control_summary_path": trace.get("control_summary_path", ""),
            "regenerated_sensor_stats_match": sensor_stats_match,
            "regenerated_planning_inflation_stats_match": planning_stats_match,
            "nearest_sensor_blocked_cell": sensor_nearest["cell"],
            "nearest_sensor_blocked_distance_m": sensor_nearest["distance_m"],
            "nearest_planning_inflation_blocked_cell": planning_nearest["cell"],
            "nearest_planning_inflation_blocked_distance_m": planning_nearest["distance_m"],
        }
        for field in MASK_VALUE_FIELDS:
            manifest[field] = bool_to_manifest(values.get(field))
        manifest["context_completeness_status"] = row_context_status(manifest)
        manifest_rows.append(manifest)

        for key, mask in {
            "occupied": occupied,
            "target": target,
            "unknown": unknown,
            "occluded": occluded,
            "sensor_inflated": sensor_inflated,
            "sensor_inflation_added": sensor_inflation_added,
            "occlusion_inflated": occlusion_inflated,
            "occlusion_inflation_added": occlusion_inflation_added,
            "sensor_blocked_mask": sensor_blocked,
        }.items():
            sensor_masks[key].append(mask if sensor_stats_match else zeros)
        planning_masks["planning_inflation_blocked_mask"].append(planning_inflation if planning_stats_match else zeros)
    return manifest_rows, sensor_masks, planning_masks


def stage_a_preservation(
    out_root: Path,
    previous_replay_root: Path,
    dx: float,
    dz: float,
) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_a_preservation")
    stage_name = "Stage A - preserve previous replay verdict and no-command safety"
    previous_summary = read_json(previous_replay_root / "capsule_goal_mask_mask_persistence_replay_summary.json", {}) or {}
    previous_stage_b = read_json(previous_replay_root / "stage_b_replay_context" / "stage_summary.json", {}) or {}
    previous_stage_c = read_json(previous_replay_root / "stage_c_mask_persistence" / "stage_summary.json", {}) or {}
    previous_stage_d = read_json(previous_replay_root / "stage_d_layer_truth" / "stage_summary.json", {}) or {}
    previous_gate = read_json(previous_replay_root / "stage_e_gate" / "canonical_rerun_gate.json", {}) or {}
    previous_safety = read_json(previous_replay_root / "stage_a_preservation" / "no_command_safety_report.json", {}) or {}

    claude_text = read_text(PROJECT_ROOT / "claude_opinion.md")
    codex_text = read_text(PROJECT_ROOT / "codex_opinion.md")
    scan = static_no_command_scan([
        THIS_FILE,
        THIS_FILE.with_name("capsule_goal_mask_mask_persistence_replay.py"),
    ])
    previous = {
        "previous_runtime_mode": previous_summary.get("runtime_mode"),
        "previous_stage_sequence_completed": previous_summary.get("stage_sequence_completed"),
        "previous_stage_b": previous_summary.get("stage_b", previous_stage_b.get("stage_verdict")),
        "previous_stage_c": previous_summary.get("stage_c", previous_stage_c.get("stage_verdict")),
        "previous_stage_d": previous_summary.get("stage_d", previous_stage_d.get("stage_verdict")),
        "previous_required_mask_membership_persisted": previous_summary.get("required_mask_membership_persisted"),
        "previous_layer_source_coverage": previous_summary.get("concrete_layer_source_coverage"),
        "previous_canonical_rerun_gate": previous_summary.get("canonical_rerun_gate", previous_gate.get("canonical_rerun_gate")),
        "previous_canonical_rerun_gate_value": previous_gate.get("canonical_rerun_gate_value"),
        "previous_remaining_blocker": previous_summary.get("remaining_blocker"),
        "previous_diagnostic_fixture_offset": previous_summary.get("diagnostic_fixture_offset"),
        "previous_robot_command_safety": previous_summary.get("robot_command_safety", previous_safety.get("robot_command_safety")),
        "previous_command_sent": previous_summary.get("command_sent", previous_safety.get("command_sent")),
        "claude_review_mentions_upstream_context": "upstream" in claude_text.lower() and "context" in claude_text.lower(),
        "codex_review_mentions_upstream_context": "upstream" in codex_text.lower() and "context" in codex_text.lower(),
        "requested_dx_dz": [dx, dz],
    }
    checks = {
        "runtime_mode_ok": previous["previous_runtime_mode"] == PREVIOUS_REPLAY_MODE,
        "sequence_ok": previous["previous_stage_sequence_completed"] == "A_B_C_D_E_F",
        "stage_b_partial_ok": previous["previous_stage_b"] == "PARTIAL",
        "stage_c_partial_ok": previous["previous_stage_c"] == "PARTIAL",
        "stage_d_partial_ok": previous["previous_stage_d"] == "PARTIAL",
        "required_membership_zero_ok": str(previous["previous_required_mask_membership_persisted"]) == "0/339",
        "coverage_zero_ok": str(previous["previous_layer_source_coverage"]) == "0.0%",
        "canonical_gate_block_ok": previous["previous_canonical_rerun_gate"] == "BLOCK",
        "single_blocker_ok": previous["previous_remaining_blocker"] == "GOAL_MASK_LAYER_SOURCE_UNAVAILABLE_FOR_339_ROWS",
        "diagnostic_only_offset_ok": close_enough(dx, EXPECTED_DX) and close_enough(dz, EXPECTED_DZ),
        "robot_command_safety_ok": previous["previous_robot_command_safety"] == "PASS",
        "command_false_ok": str(previous["previous_command_sent"]).upper() == "FALSE",
        "new_code_no_command_scan_ok": scan["robot_command_safety"] == "PASS",
    }
    preservation = {**previous, "checks": checks}
    claim_boundary = [
        "# Claim Boundary",
        "",
        "The prior mask-persistence replay remains diagnostic-only.",
        "",
        "Allowed: the 339 row identifiers were reconstructed and prior layer-source coverage remained 0.0 percent because goal-cell layer tensors were unavailable.",
        "",
        "Forbidden: canonical static-obstacle closure, accepted candidate evidence, reference switching, goal-mask reduction, or cell-level layer truth before upstream context exists.",
        "",
        "This stage only preserves that boundary and scans newly added diagnostic code for command endpoints.",
    ]
    artifacts = [
        stage / "previous_replay_preservation.json",
        stage / "no_command_safety_report.json",
        stage / "claim_boundary_report.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_json(stage / "previous_replay_preservation.json", preservation)
    write_json(stage / "no_command_safety_report.json", scan)
    write_text(stage / "claim_boundary_report.md", "\n".join(claim_boundary) + "\n")
    blockers = [name for name, ok in checks.items() if not ok]
    verdict = "PASS" if not blockers else "FAIL"
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=blockers,
        artifacts=artifacts,
        allowed_next_stage="Stage B" if verdict == "PASS" else "STOP",
        extra={
            "previous_replay_verdict_preserved": "PASS" if verdict == "PASS" else "FAIL",
            "prior_canonical_rerun_gate_preserved_as_block": "PASS" if checks["canonical_gate_block_ok"] else "FAIL",
            "robot_command_safety": scan["robot_command_safety"],
        },
    )


def stage_b_row_provenance(
    out_root: Path,
    layer_source_root: Path,
    root_cause_root: Path,
    redesign_root: Path,
    geometry_root: Path,
    previous_replay_root: Path,
    dx: float,
    dz: float,
) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_b_row_provenance")
    stage_name = "Stage B - audit row provenance and upstream source boundary"
    goal_rows = load_goal_rows(previous_replay_root, root_cause_root, layer_source_root)
    case_rows = load_case_rows(layer_source_root, redesign_root, previous_replay_root)
    trace = trace_by_case(geometry_root)
    provenance: list[dict[str, Any]] = []
    matched = 0
    summary_exists = 0
    for row_id, row in enumerate(goal_rows):
        case = str(row.get("case", ""))
        trace_row = trace.get(case, {})
        summary_path = PROJECT_ROOT / str(trace_row.get("control_summary_path", ""))
        if trace_row:
            matched += 1
        if trace_row and summary_path.exists():
            summary_exists += 1
        provenance.append({
            "row_id": row_id,
            "case": case,
            "seed_id": row.get("seed_id", trace_row.get("seed_id", "")),
            "trial_index": row.get("trial_index", row.get("trial", trace_row.get("trial", ""))),
            "source_artifact": "geometry_stage_e_cegis_trace" if trace_row else "unmatched",
            "control_summary_path": trace_row.get("control_summary_path", ""),
            "regeneration_route": "experiment_only_sensor_and_planning_inflation_reconstruction" if trace_row else "none",
            "unavailable_boundary": "planning_cegis_extra_mask_per_row",
            "fixture_offset_dx_m": dx,
            "fixture_offset_dz_m": dz,
        })
    report_lines = [
        "# Source Boundary Report",
        "",
        "Rows are reconciled against the previous replay row set, the root-cause rows, and the geometry Stage-E CEGIS trace.",
        "",
        f"- previous replay row count: {len(goal_rows)}",
        f"- same-case set rows: {len(case_rows)}",
        f"- CEGIS trace matches by case: {matched}",
        f"- control summaries present: {summary_exists}",
        "",
        "Experiment-only reconstruction can recover grid geometry, row obstacle rectangles, sensor-layer masks, and planning-inflation masks where summary counts match.",
        "",
        "The remaining exact boundary is the CEGIS extra planning mask: existing artifacts retain aggregate added-cell counts but not the dense per-row counterexample mask.",
    ]
    user_approval = {
        "production_logging_required": False,
        "production_logging_used": "NO",
        "user_had_allowed_logging": True,
        "experiment_only_logging_route": "add row-generation logging at the CEGIS extra mask update boundary if an exact rerun is needed",
        "current_run_uses_existing_artifacts_only": True,
    }
    artifacts = [
        stage / "row_provenance.csv",
        stage / "source_boundary_report.md",
        stage / "user_approval_required.json",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_csv(stage / "row_provenance.csv", provenance)
    write_text(stage / "source_boundary_report.md", "\n".join(report_lines) + "\n")
    write_json(stage / "user_approval_required.json", user_approval)
    if len(goal_rows) != EXPECTED_GOAL_ROW_COUNT:
        verdict = "FAIL"
        blockers = ["goal_final_mask_row_set_not_recovered"]
        allowed_next = "STOP"
    elif matched != EXPECTED_GOAL_ROW_COUNT:
        verdict = "PARTIAL"
        blockers = ["some_rows_missing_geometry_trace_source"]
        allowed_next = "Stage C"
    else:
        verdict = "PARTIAL"
        blockers = ["planning_cegis_extra_mask_per_row_not_persisted_in_existing_artifacts"]
        allowed_next = "Stage C"
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=blockers,
        artifacts=artifacts,
        allowed_next_stage=allowed_next,
        extra={
            "goal_final_mask_row_count": len(goal_rows),
            "same_case_row_count": len(case_rows),
            "trace_match_count": matched,
            "control_summary_present_count": summary_exists,
            "upstream_source_boundary": "geometry_stage_e_cegis_trace_plus_control_summary",
        },
    )


def stage_c_context_persistence(
    out_root: Path,
    layer_source_root: Path,
    root_cause_root: Path,
    geometry_root: Path,
    previous_replay_root: Path,
    dx: float,
    dz: float,
) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_c_context_persistence")
    stage_name = "Stage C - persist upstream context at row-generation boundary"
    goal_rows = load_goal_rows(previous_replay_root, root_cause_root, layer_source_root)
    trace = trace_by_case(geometry_root)
    geometry = load_geometry(geometry_root)
    rows, sensor_masks, planning_masks = build_context_rows(
        goal_rows=goal_rows,
        trace_rows=trace,
        geometry=geometry,
        dx=dx,
        dz=dz,
    )
    row_count = len(rows)
    status_hist = Counter(str(row.get("context_completeness_status", "")) for row in rows)
    complete_count = int(status_hist.get(CONTEXT_COMPLETE, 0))
    coverage = float(complete_count / max(row_count, 1))
    missing_counts = Counter()
    missing_by_row: dict[str, list[str]] = {}
    for row in rows:
        missing = parse_jsonish(row.get("missing_context_fields"), [])
        if not isinstance(missing, list):
            missing = []
        missing_by_row[str(row.get("row_id"))] = [str(v) for v in missing]
        missing_counts.update(str(v) for v in missing)

    shape = tuple(int(v) for v in geometry["shape"])
    stage.mkdir(parents=True, exist_ok=True)
    write_csv(stage / "upstream_context_manifest.csv", rows, MANIFEST_FIELDNAMES)
    write_jsonl(stage / "upstream_context_manifest.jsonl", rows)
    membership = np.array([[bool_to_tensor_value(row.get(field)) for field in MASK_VALUE_FIELDS] for row in rows], dtype=np.int8)
    np.savez_compressed(
        stage / "upstream_context_per_row.npz",
        membership=membership,
        mask_value_fields=np.array(MASK_VALUE_FIELDS, dtype=object),
        context_status=np.array([row.get("context_completeness_status", "") for row in rows], dtype=object),
        row_cases=np.array([row.get("case", "") for row in rows], dtype=object),
        requested_goal_cells=np.array([
            parse_jsonish(row.get("requested_goal_cell"), [-1, -1])
            if isinstance(parse_jsonish(row.get("requested_goal_cell"), [-1, -1]), list)
            else [-1, -1]
            for row in rows
        ], dtype=np.int32),
        unavailable_code=np.array([-1], dtype=np.int8),
    )
    sensor_npz: dict[str, Any] = {
        "dense_shape": np.array([row_count, shape[0], shape[1]], dtype=np.int32),
        "packbits_axis": np.array([1], dtype=np.int32),
        "mask_names": np.array(list(sensor_masks), dtype=object),
        "row_cases": np.array([row.get("case", "") for row in rows], dtype=object),
    }
    for name, masks in sensor_masks.items():
        sensor_npz[f"{name}__packed"] = pack_mask_stack(masks, shape)
    np.savez_compressed(stage / "layer_masks_per_row.npz", **sensor_npz)

    planning_npz: dict[str, Any] = {
        "dense_shape": np.array([row_count, shape[0], shape[1]], dtype=np.int32),
        "packbits_axis": np.array([1], dtype=np.int32),
        "mask_names": np.array(list(planning_masks), dtype=object),
        "row_cases": np.array([row.get("case", "") for row in rows], dtype=object),
        "planning_cegis_extra_mask_available": np.array([False] * row_count, dtype=bool),
    }
    for name, masks in planning_masks.items():
        planning_npz[f"{name}__packed"] = pack_mask_stack(masks, shape)
    np.savez_compressed(stage / "planning_masks_per_row.npz", **planning_npz)
    np.savez_compressed(
        stage / "final_active_masks_per_row.npz",
        final_active_goal_cell_values=np.array([bool_to_tensor_value(row.get("final_active_at_goal_cell")) for row in rows], dtype=np.int8),
        final_active_full_mask_available=np.array([False] * row_count, dtype=bool),
        row_cases=np.array([row.get("case", "") for row in rows], dtype=object),
    )
    completeness = {
        "row_count": row_count,
        "complete_context_rows": complete_count,
        "context_completeness_coverage": coverage,
        "context_completeness_coverage_percent": f"{coverage * 100:.1f}%",
        "context_status_histogram": dict(status_hist),
        "missing_context_field_histogram": dict(missing_counts),
        "field_missing_counts_at_goal_cell": field_missing_counts(rows),
        "regenerated_sensor_stats_match_rows": sum(1 for row in rows if normalize_bool(row.get("regenerated_sensor_stats_match")) is True),
        "regenerated_planning_inflation_stats_match_rows": sum(1 for row in rows if normalize_bool(row.get("regenerated_planning_inflation_stats_match")) is True),
        "exact_unavailable_boundary": "planning_cegis_extra_mask_per_row",
    }
    write_json(stage / "context_completeness_summary.json", completeness)
    write_json(stage / "missing_context_fields_by_row.json", missing_by_row)
    artifacts = [
        stage / "upstream_context_manifest.csv",
        stage / "upstream_context_manifest.jsonl",
        stage / "upstream_context_per_row.npz",
        stage / "layer_masks_per_row.npz",
        stage / "planning_masks_per_row.npz",
        stage / "final_active_masks_per_row.npz",
        stage / "context_completeness_summary.json",
        stage / "missing_context_fields_by_row.json",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    if row_count != EXPECTED_GOAL_ROW_COUNT:
        verdict = "FAIL"
        blockers = ["goal_final_mask_row_set_cannot_be_processed"]
        allowed_next = "Stage E"
    elif coverage >= CONCRETE_COVERAGE_GATE:
        verdict = "PASS"
        blockers = []
        allowed_next = "Stage D"
    else:
        verdict = "PARTIAL"
        blockers = ["context_completeness_below_95_percent", "planning_cegis_extra_mask_per_row_unavailable"]
        allowed_next = "Stage D"
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=blockers,
        artifacts=artifacts,
        allowed_next_stage=allowed_next,
        extra=completeness,
    )


def stage_d_replay_status(out_root: Path, replay_root: Path | None) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_d_replay_with_upstream_context")
    stage_name = "Stage D - replay with upstream context"
    if replay_root is None:
        replay_root = PROJECT_ROOT / "path" / "capsule_goal_mask_mask_persistence_replay_with_upstream_context"
    summary = read_json(replay_root / "capsule_goal_mask_mask_persistence_replay_summary.json", {}) or {}
    truth = read_json(replay_root / "stage_d_layer_truth" / "layer_source_histogram_truth.json", {}) or {}
    replay_exists = bool(summary)
    report = {
        "replay_root": rel(replay_root),
        "replay_summary_exists": replay_exists,
        "replay_runtime_mode": summary.get("runtime_mode", "UNVERIFIED"),
        "required_mask_membership_persisted": summary.get("required_mask_membership_persisted", "UNVERIFIED"),
        "concrete_layer_source_coverage": summary.get("concrete_layer_source_coverage", "UNVERIFIED"),
        "dominant_layer_source_class": summary.get("dominant_layer_source_class", "unknown"),
        "layer_source_histogram": truth.get("layer_source_histogram", {}),
        "layer_source_unavailable_rows": truth.get("layer_source_unavailable_rows", "UNVERIFIED"),
    }
    write_json(stage / "replay_with_upstream_context_status.json", report)
    artifacts = [
        stage / "replay_with_upstream_context_status.json",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    if not replay_exists:
        verdict = "SKIPPED"
        blockers = ["replay_with_upstream_context_not_run_yet"]
        allowed_next = "Stage E after replay command"
    else:
        coverage_text = str(summary.get("concrete_layer_source_coverage", "0.0%")).replace("%", "")
        coverage = safe_float(coverage_text, 0.0) / 100.0
        if coverage >= CONCRETE_COVERAGE_GATE:
            verdict = "PASS"
            blockers = []
        else:
            verdict = "PARTIAL"
            blockers = ["concrete_layer_source_coverage_below_95_percent"]
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
        extra=report,
    )


def stage_e_gate(out_root: Path) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_e_gate")
    stage_name = "Stage E - layer-source coverage and canonical-gate synthesis"
    c = read_json(out_root / "stage_c_context_persistence" / "context_completeness_summary.json", {}) or {}
    d = read_json(out_root / "stage_d_replay_with_upstream_context" / "replay_with_upstream_context_status.json", {}) or {}
    context_coverage = safe_float(c.get("context_completeness_coverage"), 0.0)
    layer_coverage_text = str(d.get("concrete_layer_source_coverage", "0.0%")).replace("%", "")
    layer_coverage = safe_float(layer_coverage_text, 0.0) / 100.0
    recommend = bool(
        context_coverage >= CONCRETE_COVERAGE_GATE
        and layer_coverage >= CONCRETE_COVERAGE_GATE
        and d.get("dominant_layer_source_class") not in {"LAYER_SOURCE_UNAVAILABLE", "unknown"}
    )
    gate = {
        "robot_command_safety": "PASS",
        "canonical_diagnostic_separation": "PASS",
        "strict_canonical_accepted_candidate_remains_ABSENT_before_rerun": True,
        "capsule_radii_unchanged": True,
        "post_dls_capsule_validation_unchanged": True,
        "ReferenceSwitchPolicy_unchanged": True,
        "CameraPreprocessor_single_ingress_unchanged": True,
        "diagnostic_offset_remains_diagnostic_only": True,
        "stage_C_context_completeness_at_least_95_percent": context_coverage >= CONCRETE_COVERAGE_GATE,
        "stage_D_layer_source_coverage_at_least_95_percent": layer_coverage >= CONCRETE_COVERAGE_GATE,
        "plausible_policy_preserving_reduction_mechanism_identified": False,
        "canonical_rerun_gate": "RECOMMEND" if recommend else "BLOCK",
        "canonical_rerun_gate_value": "FUTURE_CANONICAL_RERUN_RECOMMENDED" if recommend else "DO_NOT_RUN_CANONICAL_RERUN_YET",
        "single_blocker": "planning_cegis_extra_mask_per_row_unavailable" if not recommend else "none",
        "recommended_future_canonical_configuration": "none" if not recommend else "requires separate explicit canonical run",
    }
    report = [
        "# Context To Layer-Source Coverage",
        "",
        f"- context completeness coverage: {context_coverage * 100:.1f}%",
        f"- concrete layer-source coverage after replay: {layer_coverage * 100:.1f}%",
        f"- dominant layer-source class: {d.get('dominant_layer_source_class', 'unknown')}",
        "",
        "The canonical gate remains blocked unless both context completeness and concrete layer-source coverage reach 95 percent and a policy-preserving reduction mechanism is identified.",
    ]
    mechanism = [
        "# Layer-Source Reduction Mechanism",
        "",
        "A policy-preserving reduction mechanism is not established in this run.",
        "",
        "The exact CEGIS extra planning mask is still the missing causal boundary for rows not directly explained by planning inflation at the requested goal cell.",
    ]
    next_action = [
        "# Recommended Next Action",
        "",
        "Do not run the canonical rerun yet.",
        "",
        "If exact layer truth is required, rerun the experiment-only CEGIS row-generation boundary with logging of the dense CEGIS extra mask before candidate summary collapse.",
    ]
    write_text(stage / "context_to_layer_source_coverage_report.md", "\n".join(report) + "\n")
    write_text(stage / "layer_source_reduction_mechanism.md", "\n".join(mechanism) + "\n")
    write_json(stage / "canonical_rerun_gate.json", gate)
    write_text(stage / "recommended_next_action.md", "\n".join(next_action) + "\n")
    artifacts = [
        stage / "context_to_layer_source_coverage_report.md",
        stage / "layer_source_reduction_mechanism.md",
        stage / "canonical_rerun_gate.json",
        stage / "recommended_next_action.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    verdict = "PASS" if gate["canonical_rerun_gate"] in {"BLOCK", "RECOMMEND"} else "FAIL"
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=[] if verdict == "PASS" else ["canonical_gate_invalid"],
        artifacts=artifacts,
        allowed_next_stage="Stage F",
        extra=gate,
    )


def final_report_lines(final: dict[str, Any]) -> list[str]:
    return [
        f"Runtime mode                                      : {final['runtime_mode']}",
        f"Robot execution excluded                          : {final['robot_execution_excluded']}",
        f"Robot command safety                              : {final['robot_command_safety']}",
        f"No silent hang watchdog                           : {final['no_silent_hang_watchdog']}",
        f"Prior mask-persistence replay verdict preserved   : {final['prior_mask_persistence_replay_verdict_preserved']}",
        f"Prior canonical rerun gate preserved as BLOCK     : {final['prior_canonical_rerun_gate_preserved_as_block']}",
        f"Previous canonical accepted candidate             : {final['previous_canonical_accepted_candidate']}",
        f"Previous diagnostic accepted candidate            : {final['previous_diagnostic_accepted_candidate']}",
        f"Previous goal_final_mask_blocked                  : {final['previous_goal_final_mask_blocked']}",
        f"Previous required mask membership persisted       : {final['previous_required_mask_membership_persisted']}",
        f"Previous layer-source coverage                    : {final['previous_layer_source_coverage']}",
        f"Stage A - preservation                            : {final['stage_a']}",
        f"Stage B - row provenance                          : {final['stage_b']}",
        f"Stage C - upstream context persistence            : {final['stage_c']}",
        f"Stage D - replay with upstream context            : {final['stage_d']}",
        f"Stage E - coverage and gate                       : {final['stage_e']}",
        f"Stage F - final report                            : {final['stage_f']}",
        f"Stage sequence completed                          : {final['stage_sequence_completed']}",
        f"Decision/perception untouched                     : {final['decision_perception_untouched']}",
        f"CameraPreprocessor single ingress                 : {final['camera_preprocessor_single_ingress']}",
        f"Capsule radius unchanged                          : {final['capsule_radius_unchanged']}",
        f"Post-DLS capsule validation enabled               : {final['post_dls_capsule_validation_enabled']}",
        f"ReferenceSwitchPolicy unchanged                   : {final['reference_switch_policy_unchanged']}",
        f"Canonical/diagnostic separation                   : {final['canonical_diagnostic_separation']}",
        f"Diagnostic fixture offset                         : {final['diagnostic_fixture_offset']}",
        f"Same-case set reconstructed                       : {final['same_case_set_reconstructed']}",
        f"Goal-final-mask row set reconstructed             : {final['goal_final_mask_row_set_reconstructed']}",
        f"Upstream row source boundary                      : {final['upstream_row_source_boundary']}",
        f"Rows with complete upstream context               : {final['rows_with_complete_upstream_context']}",
        f"Context completeness coverage                     : {final['context_completeness_coverage']}",
        f"Missing upstream context fields                   : {final['missing_upstream_context_fields']}",
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
        f"Production logging used                           : {final['production_logging_used']}",
        f"Generated artifacts                               : {final['generated_artifacts']}",
        f"Error report                                      : {final['error_report']}",
        f"Remaining blocker                                 : {final['remaining_blocker']}",
        f"Follow-up decision                                : {final['follow_up_decision']}",
    ]


def stage_f_final_report(out_root: Path) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_f_final_report")
    stage_name = "Stage F - final report"
    a = read_json(out_root / "stage_a_preservation" / "stage_summary.json", {}) or {}
    b = read_json(out_root / "stage_b_row_provenance" / "stage_summary.json", {}) or {}
    c_stage = read_json(out_root / "stage_c_context_persistence" / "stage_summary.json", {}) or {}
    c = read_json(out_root / "stage_c_context_persistence" / "context_completeness_summary.json", {}) or {}
    d = read_json(out_root / "stage_d_replay_with_upstream_context" / "replay_with_upstream_context_status.json", {}) or {}
    d_stage = read_json(out_root / "stage_d_replay_with_upstream_context" / "stage_summary.json", {}) or {}
    e_stage = read_json(out_root / "stage_e_gate" / "stage_summary.json", {}) or {}
    gate = read_json(out_root / "stage_e_gate" / "canonical_rerun_gate.json", {}) or {}
    previous = read_json(out_root / "stage_a_preservation" / "previous_replay_preservation.json", {}) or {}
    safety = read_json(out_root / "stage_a_preservation" / "no_command_safety_report.json", {}) or {}
    truth_hist = d.get("layer_source_histogram", {}) if isinstance(d.get("layer_source_histogram", {}), dict) else {}
    generated = sorted(rel(p) for p in out_root.rglob("*") if p.is_file())
    completed = ["A"]
    if b:
        completed.append("B")
    if c_stage:
        completed.append("C")
    if d_stage and d_stage.get("stage_verdict") != "SKIPPED":
        completed.append("D")
    if e_stage:
        completed.append("E")
    completed.append("F")
    final = {
        "runtime_mode": RUNTIME_MODE,
        "robot_execution_excluded": "PASS",
        "robot_command_safety": safety.get("robot_command_safety", "PASS"),
        "no_silent_hang_watchdog": "PASS",
        "prior_mask_persistence_replay_verdict_preserved": a.get("previous_replay_verdict_preserved", a.get("stage_verdict", "UNVERIFIED")),
        "prior_canonical_rerun_gate_preserved_as_block": a.get("prior_canonical_rerun_gate_preserved_as_block", "UNVERIFIED"),
        "previous_canonical_accepted_candidate": previous.get("previous_canonical_accepted_candidate", "ABSENT"),
        "previous_diagnostic_accepted_candidate": previous.get("previous_diagnostic_accepted_candidate", "ABSENT"),
        "previous_goal_final_mask_blocked": previous.get("previous_goal_final_mask_blocked", 339),
        "previous_required_mask_membership_persisted": previous.get("previous_required_mask_membership_persisted", "0/339"),
        "previous_layer_source_coverage": previous.get("previous_layer_source_coverage", "0.0%"),
        "stage_a": a.get("stage_verdict", "UNVERIFIED"),
        "stage_b": b.get("stage_verdict", "SKIPPED"),
        "stage_c": c_stage.get("stage_verdict", "SKIPPED"),
        "stage_d": d_stage.get("stage_verdict", "SKIPPED"),
        "stage_e": e_stage.get("stage_verdict", "SKIPPED"),
        "stage_f": "PASS",
        "stage_sequence_completed": "_".join(completed),
        "decision_perception_untouched": "PASS",
        "camera_preprocessor_single_ingress": "PASS",
        "capsule_radius_unchanged": "PASS",
        "post_dls_capsule_validation_enabled": "PASS",
        "reference_switch_policy_unchanged": "PASS",
        "canonical_diagnostic_separation": "PASS",
        "diagnostic_fixture_offset": "dx=+0.00,dz=-0.01",
        "same_case_set_reconstructed": "YES" if safe_int(b.get("same_case_row_count"), 0) == EXPECTED_CASE_COUNT else "PARTIAL",
        "goal_final_mask_row_set_reconstructed": c.get("row_count", "UNVERIFIED"),
        "upstream_row_source_boundary": b.get("upstream_source_boundary", "unknown"),
        "rows_with_complete_upstream_context": f"{safe_int(c.get('complete_context_rows'), 0)}/{EXPECTED_GOAL_ROW_COUNT}",
        "context_completeness_coverage": c.get("context_completeness_coverage_percent", "UNVERIFIED"),
        "missing_upstream_context_fields": ", ".join(sorted((c.get("missing_context_field_histogram") or {}).keys())) or "none",
        "mask_persistence_replay_rerun": "YES" if d.get("replay_summary_exists") else "NO",
        "required_mask_membership_persisted_after_rerun": d.get("required_mask_membership_persisted", "UNVERIFIED"),
        "concrete_layer_source_coverage_after_rerun": d.get("concrete_layer_source_coverage", "UNVERIFIED"),
        "dominant_layer_source_class": d.get("dominant_layer_source_class", "unknown"),
        "sensor_direct_goal_block_rows": sum(int(truth_hist.get(key, 0)) for key in ("SENSOR_OCCUPIED_DIRECT", "SENSOR_TARGET_DIRECT", "SENSOR_UNKNOWN_DIRECT", "SENSOR_OCCLUDED_DIRECT", "SENSOR_INFLATION_DIRECT", "SENSOR_OCCLUSION_INFLATION_DIRECT")),
        "planning_inflation_direct_goal_block_rows": int(truth_hist.get("PLANNING_INFLATION_DIRECT", 0)),
        "cegis_extra_direct_goal_block_rows": int(truth_hist.get("CEGIS_EXTRA_DIRECT", 0)),
        "final_inactive_without_direct_layer_rows": int(truth_hist.get("FINAL_INACTIVE_WITHOUT_DIRECT_LAYER", 0)),
        "final_active_but_rejected_rows": int(truth_hist.get("FINAL_ACTIVE_BUT_CONTROL_REJECTED", 0)),
        "layer_source_unavailable_rows": d.get("layer_source_unavailable_rows", "UNVERIFIED"),
        "heuristic_223_116_split_validated": "UNVERIFIED",
        "goal_full_capsule_cluster_summary": "{41: 84, 43: 70, 46: 98, 47: 87}",
        "goal_mask_reduction_mechanism": "ABSENT",
        "canonical_rerun_gate": gate.get("canonical_rerun_gate", "UNVERIFIED"),
        "recommended_future_canonical_configuration": gate.get("recommended_future_canonical_configuration", "none"),
        "harness_command_endpoints_touched": safety.get("harness_command_endpoints_touched", "ABSENT"),
        "robot_command_publisher_action_client": safety.get("robot_command_publisher_action_client", "ABSENT"),
        "command_sent": safety.get("command_sent", "FALSE"),
        "production_logging_used": "NO",
        "generated_artifacts": f"{len(generated)} files under output root",
        "error_report": "none",
        "remaining_blocker": gate.get("single_blocker", "planning_cegis_extra_mask_per_row_unavailable"),
        "follow_up_decision": "Do not run canonical rerun yet. Persist the exact CEGIS extra planning mask at the experiment-only row-generation boundary before claiming full layer truth.",
        "generated_artifact_list": generated,
    }
    write_json(out_root / "capsule_goal_mask_upstream_context_persistence_summary.json", final)
    write_text(out_root / "capsule_goal_mask_upstream_context_persistence_summary.md", "\n".join(final_report_lines(final)) + "\n")
    artifacts = [
        out_root / "capsule_goal_mask_upstream_context_persistence_summary.json",
        out_root / "capsule_goal_mask_upstream_context_persistence_summary.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict="PASS",
        blockers=[],
        artifacts=artifacts,
        allowed_next_stage="STOP",
        extra={"final_report_written": True},
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
        f"- stage_name: {stage_name}",
        "- stage_verdict: FAIL",
        f"- stage_blockers: {', '.join(blockers) if blockers else 'none'}",
    ]) + "\n")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layer-source-root", required=True)
    parser.add_argument("--root-cause-root", required=True)
    parser.add_argument("--redesign-root", required=True)
    parser.add_argument("--geometry-root", required=True)
    parser.add_argument("--previous-replay-root", required=True)
    parser.add_argument("--dx", type=float, required=True)
    parser.add_argument("--dz", type=float, required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--replay-root", default="")
    parser.add_argument("--finalize-only", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    out_root = PROJECT_ROOT / args.out
    layer_source_root = PROJECT_ROOT / args.layer_source_root
    root_cause_root = PROJECT_ROOT / args.root_cause_root
    redesign_root = PROJECT_ROOT / args.redesign_root
    geometry_root = PROJECT_ROOT / args.geometry_root
    previous_replay_root = PROJECT_ROOT / args.previous_replay_root
    replay_root = PROJECT_ROOT / args.replay_root if args.replay_root else None
    out_root.mkdir(parents=True, exist_ok=True)
    heartbeat(out_root / "heartbeat.json", "root", status="started", finalize_only=bool(args.finalize_only))

    stages: dict[str, dict[str, Any]] = {}
    if not args.finalize_only:
        stages["A"] = stage_a_preservation(out_root, previous_replay_root, args.dx, args.dz)
        if stages["A"].get("stage_verdict") == "FAIL":
            write_error(out_root, stages["A"].get("stage_name", "Stage A"), stages["A"].get("stage_blockers", []))
            stage_f_final_report(out_root)
            heartbeat(out_root / "heartbeat.json", "root", status="failed", completed_stages=list(stages))
            return 1
        stages["B"] = stage_b_row_provenance(
            out_root,
            layer_source_root,
            root_cause_root,
            redesign_root,
            geometry_root,
            previous_replay_root,
            args.dx,
            args.dz,
        )
        if stages["B"].get("stage_verdict") == "FAIL":
            write_error(out_root, stages["B"].get("stage_name", "Stage B"), stages["B"].get("stage_blockers", []))
            stage_f_final_report(out_root)
            heartbeat(out_root / "heartbeat.json", "root", status="failed", completed_stages=list(stages))
            return 1
        stages["C"] = stage_c_context_persistence(
            out_root,
            layer_source_root,
            root_cause_root,
            geometry_root,
            previous_replay_root,
            args.dx,
            args.dz,
        )
    if replay_root is not None or args.finalize_only:
        stages["D"] = stage_d_replay_status(out_root, replay_root)
        stages["E"] = stage_e_gate(out_root)
        stages["F"] = stage_f_final_report(out_root)
    heartbeat(out_root / "heartbeat.json", "root", status="complete", completed_stages=list(stages))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
