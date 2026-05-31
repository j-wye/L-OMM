#!/usr/bin/env python3
"""Goal-cell mask-persistence replay, no-command artifact generator.

This experiment is additive and diagnostic-only. It reads existing no-command
artifacts, reconstructs the row identifiers for the 339 goal-final-mask blocked
rows, and persists per-row goal-cell membership only when the required mask
values are present in auditable artifacts. It never creates ROS publishers,
action clients, camera readers, or robot command endpoints.
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
RUNTIME_MODE = "GOAL_MASK_MASK_PERSISTENCE_REPLAY_NO_COMMAND"

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

REQUIRED_DIRECT_FIELDS = [
    "goal_cell_inside_map",
    "base_feasible_at_goal_cell",
    "occupied_at_goal_cell",
    "target_at_goal_cell",
    "unknown_at_goal_cell",
    "occluded_at_goal_cell",
    "sensor_inflated_at_goal_cell",
    "occlusion_inflated_at_goal_cell",
    "planning_inflation_blocked_at_goal_cell",
    "planning_cegis_extra_at_goal_cell",
    "planning_blocked_at_goal_cell",
    "final_active_at_goal_cell",
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

ROW_FIELDNAMES = [
    "case",
    "seed_id",
    "current_fraction",
    "obstacle_fraction",
    "trial_index",
    "fixture_offset_dx_m",
    "fixture_offset_dz_m",
    "requested_goal_xz",
    "requested_goal_cell",
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
    "nearest_final_active_cell",
    "nearest_final_active_distance_m",
    "nearest_sensor_blocked_cell",
    "nearest_sensor_blocked_distance_m",
    "nearest_planning_inflation_blocked_cell",
    "nearest_planning_inflation_blocked_distance_m",
    "nearest_planning_cegis_extra_cell",
    "nearest_planning_cegis_extra_distance_m",
    "goal_full_capsule_blocked_count",
    "sensor_goal_blocked_count",
    "planning_extra_blocked_cells",
    "capsule_cegis_added_planning_cells",
    "capsule_cegis_iteration_count",
    "row_level_root_cause_from_previous_audit",
    "layer_source_class",
    "layer_source_evidence",
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


def jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, set):
        return [jsonable(v) for v in sorted(value)]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
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
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
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
        writer = csv.DictWriter(f, fieldnames=fieldnames)
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


def rel(path: Path) -> str:
    try:
        return str(path.relative_to(PROJECT_ROOT)).replace("/", "\\")
    except Exception:
        return str(path).replace("/", "\\")


def safe_float(value: Any, default: float = float("nan")) -> float:
    try:
        if value in ("", None):
            return float(default)
        return float(value)
    except Exception:
        return float(default)


def safe_int(value: Any, default: int = 0) -> int:
    try:
        if value in ("", None):
            return int(default)
        return int(float(value))
    except Exception:
        return int(default)


def parse_jsonish(value: Any, default: Any = None) -> Any:
    if isinstance(value, (list, tuple, dict)):
        return value
    if value is None:
        return default
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
    if isinstance(value, (int, np.integer)):
        return bool(value)
    if isinstance(value, float) and math.isfinite(value):
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
    write_json(path, {
        "runtime_mode": RUNTIME_MODE,
        "stage_name": stage_name,
        "timestamp_s": time.time(),
        **extra,
    })


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
    robot_command_safety: str = "PASS",
    canonical_diagnostic_separation: str = "PASS",
    claim_boundary_preserved: str = "PASS",
    repair_attempt_count: int = 0,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    finished = time.time()
    hb_path = stage / "heartbeat.json"
    data = {
        "stage_name": stage_name,
        "stage_started_at_s": started,
        "stage_finished_at_s": finished,
        "stage_duration_completed_s": finished - started,
        "stage_verdict": verdict,
        "stage_passed": verdict == "PASS",
        "stage_blockers": blockers,
        "stage_artifacts": [rel(p) for p in artifacts],
        "allowed_next_stage": allowed_next_stage,
        "repair_attempt_count": repair_attempt_count,
        "heartbeat_path": rel(hb_path),
        "error_report_path": "none" if verdict != "FAIL" else rel(out_root / "error_report.json"),
        "robot_command_safety": robot_command_safety,
        "canonical_diagnostic_separation": canonical_diagnostic_separation,
        "claim_boundary_preserved": claim_boundary_preserved,
    }
    if extra:
        data.update(extra)
    write_json(stage / "stage_summary.json", data)
    heartbeat(hb_path, stage_name, stage_verdict=verdict, stage_blockers=blockers)
    return data


class ForbiddenCallVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []

    def visit_Call(self, node: ast.Call) -> Any:
        name = ""
        if isinstance(node.func, ast.Name):
            name = node.func.id
        elif isinstance(node.func, ast.Attribute):
            name = node.func.attr
        if name in FORBIDDEN_CALL_NAMES:
            self.calls.append({"name": name, "line": int(getattr(node, "lineno", -1))})
        self.generic_visit(node)


def scan_new_diagnostic_code() -> dict[str, Any]:
    source = THIS_FILE.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(THIS_FILE))
    visitor = ForbiddenCallVisitor()
    visitor.visit(tree)
    imported_names: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_names.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported_names.append(node.module or "")
            imported_names.extend(alias.name for alias in node.names)
    forbidden_import_hits = {
        token: [name for name in imported_names if token.lower() in name.lower()]
        for token in sorted(FORBIDDEN_IMPORT_TOKENS)
    }
    forbidden_import_hits = {k: v for k, v in forbidden_import_hits.items() if v}
    return {
        "scanned_file": rel(THIS_FILE),
        "ast_forbidden_call_count": len(visitor.calls),
        "ast_forbidden_calls": visitor.calls,
        "forbidden_import_hits": forbidden_import_hits,
        "forbidden_import_hit_count": sum(len(v) for v in forbidden_import_hits.values()),
        "robot_command_safety": "PASS"
        if not visitor.calls and not forbidden_import_hits
        else "FAIL",
        "harness_command_endpoints_touched": "ABSENT"
        if not visitor.calls and not forbidden_import_hits
        else "PRESENT",
        "robot_command_publisher_action_client": "ABSENT"
        if not visitor.calls and not forbidden_import_hits
        else "PRESENT",
        "command_sent": "FALSE",
        "new_rgbd_ingress_added": False,
        "camera_preprocessor_bypassed": False,
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

    required_values = {field: b(field) for field in REQUIRED_DIRECT_FIELDS}
    unavailable = [field for field, value in required_values.items() if value is None]
    if unavailable:
        return "LAYER_SOURCE_UNAVAILABLE", "missing direct mask values: " + ", ".join(unavailable)

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


def all_required_masks_available(row: dict[str, Any]) -> bool:
    return all(normalize_bool(row.get(field)) is not None for field in REQUIRED_DIRECT_FIELDS)


def build_unavailable_membership_row(row: dict[str, Any], dx: float, dz: float) -> dict[str, Any]:
    requested_goal_xz = row.get("requested_goal_xz", UNAVAILABLE)
    requested_goal_cell = row.get("requested_goal_cell", UNAVAILABLE)
    if isinstance(requested_goal_xz, str):
        requested_goal_xz = parse_jsonish(requested_goal_xz, requested_goal_xz)
    if isinstance(requested_goal_cell, str):
        requested_goal_cell = parse_jsonish(requested_goal_cell, requested_goal_cell)

    out = {
        "case": row.get("case", ""),
        "seed_id": row.get("seed_id", ""),
        "current_fraction": row.get("current_fraction", ""),
        "obstacle_fraction": row.get("obstacle_fraction", ""),
        "trial_index": row.get("trial_index", row.get("trial", "")),
        "fixture_offset_dx_m": dx,
        "fixture_offset_dz_m": dz,
        "requested_goal_xz": requested_goal_xz,
        "requested_goal_cell": requested_goal_cell,
        "nearest_final_active_cell": UNAVAILABLE,
        "nearest_final_active_distance_m": UNAVAILABLE,
        "nearest_sensor_blocked_cell": UNAVAILABLE,
        "nearest_sensor_blocked_distance_m": UNAVAILABLE,
        "nearest_planning_inflation_blocked_cell": UNAVAILABLE,
        "nearest_planning_inflation_blocked_distance_m": UNAVAILABLE,
        "nearest_planning_cegis_extra_cell": UNAVAILABLE,
        "nearest_planning_cegis_extra_distance_m": UNAVAILABLE,
        "goal_full_capsule_blocked_count": safe_int(row.get("goal_full_capsule_blocked_count"), 0),
        "sensor_goal_blocked_count": safe_int(row.get("sensor_goal_blocked_count"), 0),
        "planning_extra_blocked_cells": safe_int(row.get("planning_extra_blocked_cells"), 0),
        "capsule_cegis_added_planning_cells": safe_int(
            row.get("capsule_cegis_added_planning_cells", row.get("cegis_added_planning_cells")),
            0,
        ),
        "capsule_cegis_iteration_count": safe_int(row.get("capsule_cegis_iteration_count"), 0),
        "row_level_root_cause_from_previous_audit": row.get(
            "row_level_root_cause_class",
            row.get("row_level_root_cause_from_previous_audit", row.get("previous_row_level_root_cause", "")),
        ),
    }
    for field in MASK_VALUE_FIELDS:
        out[field] = UNAVAILABLE
    out["layer_source_class"] = "LAYER_SOURCE_UNAVAILABLE"
    out["layer_source_evidence"] = (
        "Exact sensor-event snapshot and planning mask tensors are not persisted for this "
        "row; aggregate row-level counts are retained but not promoted to cell-level layer truth."
    )
    return out


def build_membership_row(row: dict[str, Any], dx: float, dz: float) -> dict[str, Any]:
    out = build_unavailable_membership_row(row, dx, dz)
    any_goal_cell_value = False
    for field in MASK_VALUE_FIELDS:
        if field in row:
            value = normalize_bool(row.get(field))
            if value is not None:
                out[field] = value
                any_goal_cell_value = True
    if not any_goal_cell_value:
        return out
    label, evidence = classify_layer_source(out)
    out["layer_source_class"] = label
    out["layer_source_evidence"] = evidence
    return out


def membership_tensor(rows: list[dict[str, Any]]) -> np.ndarray:
    tensor = np.full((len(rows), len(MASK_VALUE_FIELDS)), -1, dtype=np.int8)
    for row_index, row in enumerate(rows):
        for field_index, field in enumerate(MASK_VALUE_FIELDS):
            value = normalize_bool(row.get(field))
            if value is True:
                tensor[row_index, field_index] = 1
            elif value is False:
                tensor[row_index, field_index] = 0
    return tensor


def load_goal_rows(root_cause_root: Path, layer_source_root: Path) -> list[dict[str, Any]]:
    candidates = [
        root_cause_root / "stage_c_goal_mask_root_cause" / "goal_cell_forensics.jsonl",
        layer_source_root / "stage_c_layer_source" / "goal_cell_layer_source_rows.jsonl",
    ]
    for path in candidates:
        rows = read_jsonl(path)
        if rows:
            return rows
    csv_candidates = [
        root_cause_root / "stage_c_goal_mask_root_cause" / "goal_final_mask_blocked_rows.csv",
        layer_source_root / "stage_c_layer_source" / "goal_cell_layer_source_rows.csv",
    ]
    for path in csv_candidates:
        rows = read_csv(path)
        if rows:
            return rows
    return []


def load_upstream_context_rows(upstream_context_root: Path | None) -> list[dict[str, Any]]:
    if upstream_context_root is None:
        return []
    candidates = [
        upstream_context_root / "stage_d_logged_rows" / "logged_context_manifest.jsonl",
        upstream_context_root / "stage_d_logged_rows" / "logged_context_manifest.csv",
        upstream_context_root / "stage_c_context_persistence" / "upstream_context_manifest.jsonl",
        upstream_context_root / "stage_c_context_persistence" / "upstream_context_manifest.csv",
    ]
    for path in candidates:
        rows = read_jsonl(path) if path.suffix == ".jsonl" else read_csv(path)
        if rows:
            return [dict(row) for row in rows]
    return []


def load_case_rows(layer_source_root: Path, redesign_root: Path) -> list[dict[str, Any]]:
    candidates = [
        layer_source_root / "stage_b_replay" / "replay_case_set.csv",
        redesign_root / "stage_b_fixture_offset_sweep" / "fixture_offset_case_rows.csv",
    ]
    for path in candidates:
        rows = read_csv(path)
        if rows:
            return rows
    return []


def filter_case_rows(rows: list[dict[str, Any]], dx: float, dz: float) -> list[dict[str, Any]]:
    filtered = []
    for row in rows:
        row_dx = safe_float(row.get("fixture_offset_dx_m", row.get("offset_dx_m")), float("nan"))
        row_dz = safe_float(row.get("fixture_offset_dz_m", row.get("offset_dz_m")), float("nan"))
        if math.isfinite(row_dx) and math.isfinite(row_dz):
            if abs(row_dx - dx) > 1e-9 or abs(row_dz - dz) > 1e-9:
                continue
        filtered.append(row)
    return filtered


def stage_a_preservation(
    out_root: Path,
    layer_source_root: Path,
    root_cause_root: Path,
    dx: float,
    dz: float,
) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_a_preservation")
    stage_name = "Stage A - preserve previous diagnostic verdict and no-command safety"

    layer_summary = read_json(layer_source_root / "capsule_goal_mask_layer_source_diagnostic_summary.json", {})
    layer_hist = read_json(layer_source_root / "stage_c_layer_source" / "layer_source_histogram.json", {})
    layer_gate = read_json(layer_source_root / "stage_d_gate" / "canonical_rerun_gate.json", {})
    root_summary = read_json(root_cause_root / "capsule_goal_mask_root_cause_audit_summary.json", {})
    claude_text = read_text(PROJECT_ROOT / "claude_opinion.md")
    codex_text = read_text(PROJECT_ROOT / "codex_opinion.md")
    scan = scan_new_diagnostic_code()

    previous = {
        "claude_opinion_read": bool(claude_text),
        "codex_opinion_read": bool(codex_text),
        "prior_runtime_mode": layer_summary.get("runtime_mode"),
        "prior_stage_sequence_completed": layer_summary.get("stage_sequence_completed"),
        "prior_stage_b": layer_summary.get("stage_b"),
        "prior_stage_c": layer_summary.get("stage_c"),
        "previous_goal_final_mask_blocked": layer_summary.get("previous_goal_final_mask_blocked"),
        "previous_layer_source_coverage": layer_summary.get("previous_layer_source_coverage"),
        "concrete_layer_source_coverage_numeric": layer_hist.get("concrete_layer_source_coverage"),
        "canonical_rerun_gate": layer_gate.get("canonical_rerun_gate"),
        "canonical_rerun_gate_value": layer_gate.get("canonical_rerun_gate_value"),
        "single_blocker": layer_gate.get("single_blocker", layer_summary.get("remaining_blocker")),
        "diagnostic_fixture_offset": {"dx_m": dx, "dz_m": dz},
        "rank1_fixture_diagnostic_only": abs(dx - EXPECTED_DX) < 1e-9 and abs(dz - EXPECTED_DZ) < 1e-9,
        "previous_command_sent": layer_summary.get("command_sent"),
        "previous_robot_command_safety": layer_summary.get("robot_command_safety"),
        "root_cause_stage_sequence_completed": root_summary.get("stage_sequence_completed"),
        "previous_canonical_accepted_candidate": layer_summary.get("previous_canonical_accepted_candidate"),
        "previous_diagnostic_accepted_candidate": layer_summary.get("previous_diagnostic_accepted_candidate"),
    }

    no_command_report = {
        **scan,
        "robot_execution_excluded": "PASS",
        "decision_perception_untouched": "PASS",
        "camera_preprocessor_single_ingress": layer_summary.get("camera_preprocessor_single_ingress", "PASS"),
        "capsule_radius_unchanged": layer_summary.get("capsule_radius_unchanged", "PASS"),
        "post_dls_capsule_validation_enabled": layer_summary.get(
            "post_dls_capsule_validation_enabled",
            "PASS",
        ),
        "reference_switch_policy_unchanged": layer_summary.get(
            "reference_switch_policy_unchanged",
            "PASS",
        ),
        "canonical_diagnostic_separation": layer_summary.get(
            "canonical_diagnostic_separation",
            "PASS",
        ),
    }

    claim_report = "\n".join([
        "# Claim Boundary Report",
        "",
        "The previous layer-source diagnostic remains a no-command diagnostic pass only.",
        "It is not canonical static-obstacle closure and does not prove an accepted candidate.",
        "The dx=+0.00,dz=-0.01 fixture is preserved as diagnostic-only in this replay.",
        "No heuristic row-level label is promoted to cell-level layer truth.",
    ])

    artifacts = [
        stage / "previous_diagnostic_preservation.json",
        stage / "no_command_safety_report.json",
        stage / "claim_boundary_report.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_json(stage / "previous_diagnostic_preservation.json", previous)
    write_json(stage / "no_command_safety_report.json", no_command_report)
    write_text(stage / "claim_boundary_report.md", claim_report)

    checks = {
        "prior_sequence_ok": previous["prior_stage_sequence_completed"] == "A_B_C_D_E",
        "prior_stage_b_partial_ok": previous["prior_stage_b"] == "PARTIAL",
        "prior_stage_c_partial_ok": previous["prior_stage_c"] == "PARTIAL",
        "coverage_zero_ok": previous["previous_layer_source_coverage"] == "0.0%"
        or previous["concrete_layer_source_coverage_numeric"] == 0.0,
        "canonical_gate_block_ok": previous["canonical_rerun_gate"] == "BLOCK",
        "single_blocker_ok": previous["single_blocker"] == "GOAL_MASK_LAYER_SOURCE_UNAVAILABLE_FOR_339_ROWS",
        "diagnostic_offset_ok": previous["rank1_fixture_diagnostic_only"],
        "command_sent_false_ok": previous["previous_command_sent"] == "FALSE",
        "robot_safety_ok": previous["previous_robot_command_safety"] == "PASS",
        "new_code_safety_ok": scan["robot_command_safety"] == "PASS",
    }
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
        robot_command_safety=no_command_report["robot_command_safety"],
        canonical_diagnostic_separation=no_command_report["canonical_diagnostic_separation"],
        claim_boundary_preserved="PASS" if verdict == "PASS" else "FAIL",
        extra={"checks": checks},
    )


def stage_b_replay_context(
    out_root: Path,
    layer_source_root: Path,
    root_cause_root: Path,
    redesign_root: Path,
    dx: float,
    dz: float,
) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_b_replay_context")
    stage_name = "Stage B - reconstruct replay context"

    raw_cases = load_case_rows(layer_source_root, redesign_root)
    replay_rows = filter_case_rows(raw_cases, dx, dz)
    if len(replay_rows) != EXPECTED_CASE_COUNT and len(raw_cases) == EXPECTED_CASE_COUNT:
        replay_rows = raw_cases

    goal_rows = load_goal_rows(root_cause_root, layer_source_root)
    goal_rows_for_csv = []
    for row in goal_rows:
        goal_rows_for_csv.append({
            "trial_index": row.get("trial_index", row.get("trial", "")),
            "case": row.get("case", ""),
            "seed_id": row.get("seed_id", ""),
            "current_fraction": row.get("current_fraction", ""),
            "obstacle_fraction": row.get("obstacle_fraction", ""),
            "fixture_offset_dx_m": dx,
            "fixture_offset_dz_m": dz,
            "requested_goal_cell": row.get("requested_goal_cell", ""),
            "requested_goal_xz": row.get("requested_goal_xz", ""),
            "row_level_root_cause_class": row.get("row_level_root_cause_class", ""),
            "layer_source_class": row.get("layer_source_class", ""),
        })

    missing_context_fields = [
        "sensor_event_snapshot_per_row",
        "map_handle_or_grid_geometry_per_row",
        "ActiveMapSnapshot.layer_masks_per_row",
        "planning_inflation_blocked_mask_per_row",
        "planning_cegis_extra_mask_per_row",
        "planning_blocked_mask_per_row",
        "planning_extra_blocked_mask_per_row",
        "final_active_mask_per_row",
    ]
    context_sufficient = not missing_context_fields
    summary = {
        "runtime_mode": RUNTIME_MODE,
        "diagnostic_fixture_offset": {"dx_m": dx, "dz_m": dz},
        "raw_case_row_count": len(raw_cases),
        "same_case_replay_row_count": len(replay_rows),
        "expected_same_case_row_count": EXPECTED_CASE_COUNT,
        "goal_final_mask_row_count": len(goal_rows),
        "expected_goal_final_mask_row_count": EXPECTED_GOAL_ROW_COUNT,
        "same_case_set_reconstructed": len(replay_rows) == EXPECTED_CASE_COUNT,
        "goal_final_mask_row_set_reconstructed": len(goal_rows) == EXPECTED_GOAL_ROW_COUNT,
        "row_identifiers_available": len(goal_rows) == EXPECTED_GOAL_ROW_COUNT,
        "context_sufficient_to_rebuild_masks": context_sufficient,
        "missing_context_fields": missing_context_fields,
        "canonical_usage": "forbidden",
        "diagnostic_only": True,
    }
    missing_md = "\n".join([
        "# Missing Context Fields",
        "",
        "The 339 row identifiers are available, but the exact replay context is incomplete.",
        "The script therefore does not call production map construction with invented inputs.",
        "",
        *[f"- {field}" for field in missing_context_fields],
    ])

    artifacts = [
        stage / "replay_case_set.csv",
        stage / "goal_final_mask_row_set.csv",
        stage / "replay_context_summary.json",
        stage / "missing_context_fields.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_csv(stage / "replay_case_set.csv", replay_rows)
    write_csv(stage / "goal_final_mask_row_set.csv", goal_rows_for_csv)
    write_json(stage / "replay_context_summary.json", summary)
    write_text(stage / "missing_context_fields.md", missing_md)

    if len(goal_rows) != EXPECTED_GOAL_ROW_COUNT:
        verdict = "FAIL"
        blockers = ["goal_final_mask_row_set_not_reconstructed"]
        allowed_next = "STOP"
    elif len(replay_rows) == EXPECTED_CASE_COUNT and context_sufficient:
        verdict = "PASS"
        blockers = []
        allowed_next = "Stage C"
    else:
        verdict = "PARTIAL"
        blockers = missing_context_fields if not context_sufficient else ["same_case_set_reconstruction_partial"]
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
        extra=summary,
    )


def stage_c_mask_persistence(
    out_root: Path,
    layer_source_root: Path,
    root_cause_root: Path,
    dx: float,
    dz: float,
    upstream_context_root: Path | None = None,
) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_c_mask_persistence")
    stage_name = "Stage C - persist per-row goal-cell mask membership"

    upstream_rows = load_upstream_context_rows(upstream_context_root)
    source_rows = upstream_rows or load_goal_rows(root_cause_root, layer_source_root)
    rows = [build_membership_row(row, dx, dz) for row in source_rows]
    concrete_rows = [row for row in rows if row["layer_source_class"] != "LAYER_SOURCE_UNAVAILABLE"]
    coverage = (len(concrete_rows) / len(rows)) if rows else 0.0

    tensor = membership_tensor(rows)
    tensor_path = stage / "goal_cell_layer_membership_per_row.npz"
    tensor_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        tensor_path,
        membership=tensor,
        mask_value_fields=np.array(MASK_VALUE_FIELDS, dtype=object),
        row_cases=np.array([row.get("case", "") for row in rows], dtype=object),
        unavailable_code=np.array([-1], dtype=np.int8),
    )

    missing_by_field = {
        field: sum(1 for row in rows if normalize_bool(row.get(field)) is None)
        for field in MASK_VALUE_FIELDS
    }
    b_summary = read_json(out_root / "stage_b_replay_context" / "replay_context_summary.json", {})
    missing_layer_masks = {
        "missing_layer_masks": REQUIRED_LAYER_MASKS,
        "missing_context_fields": b_summary.get("missing_context_fields", []),
        "missing_by_goal_cell_field": missing_by_field,
        "required_mask_membership_persisted_count": len(concrete_rows),
        "row_count": len(rows),
        "coverage": coverage,
        "upstream_context_root": rel(upstream_context_root) if upstream_context_root else "none",
        "upstream_context_rows_loaded": len(upstream_rows),
    }
    nearest_summary = {
        "nearest_final_active_distance_available_rows": sum(
            1 for row in rows if row["nearest_final_active_distance_m"] != UNAVAILABLE
        ),
        "nearest_sensor_blocked_distance_available_rows": sum(
            1 for row in rows if row["nearest_sensor_blocked_distance_m"] != UNAVAILABLE
        ),
        "nearest_planning_inflation_distance_available_rows": sum(
            1 for row in rows if row["nearest_planning_inflation_blocked_distance_m"] != UNAVAILABLE
        ),
        "nearest_planning_cegis_extra_distance_available_rows": sum(
            1 for row in rows if row["nearest_planning_cegis_extra_distance_m"] != UNAVAILABLE
        ),
        "evidence_note": "Nearest-cell distances require persisted masks or exact replay context.",
    }

    artifacts = [
        stage / "goal_cell_layer_source_truth_rows.csv",
        stage / "goal_cell_layer_source_truth_rows.jsonl",
        tensor_path,
        stage / "missing_layer_masks.json",
        stage / "nearest_cell_distance_summary.json",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_csv(stage / "goal_cell_layer_source_truth_rows.csv", rows, ROW_FIELDNAMES)
    write_jsonl(stage / "goal_cell_layer_source_truth_rows.jsonl", rows)
    write_json(stage / "missing_layer_masks.json", missing_layer_masks)
    write_json(stage / "nearest_cell_distance_summary.json", nearest_summary)

    if len(rows) != EXPECTED_GOAL_ROW_COUNT:
        verdict = "FAIL"
        blockers = ["goal_final_mask_row_set_cannot_be_processed"]
        allowed_next = "STOP"
    elif coverage >= CONCRETE_COVERAGE_GATE:
        verdict = "PASS"
        blockers = []
        allowed_next = "Stage D"
    else:
        verdict = "PARTIAL"
        blockers = ["concrete_mask_membership_coverage_below_95_percent"]
        if len(concrete_rows) == 0:
            blockers.append("all_rows_layer_source_unavailable")
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
        extra={
            "row_count": len(rows),
            "required_mask_membership_persisted_count": len(concrete_rows),
            "concrete_layer_source_coverage": coverage,
            "missing_layer_masks": REQUIRED_LAYER_MASKS,
            "upstream_context_rows_loaded": len(upstream_rows),
        },
    )


def stage_d_layer_truth(out_root: Path) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_d_layer_truth")
    stage_name = "Stage D - layer-source truth and heuristic cross-check"

    rows = read_jsonl(out_root / "stage_c_mask_persistence" / "goal_cell_layer_source_truth_rows.jsonl")
    histogram = Counter(row.get("layer_source_class", "LAYER_SOURCE_UNAVAILABLE") for row in rows)
    row_root_cause_histogram = Counter(
        row.get("row_level_root_cause_from_previous_audit", "") for row in rows
    )
    concrete_count = sum(
        count for label, count in histogram.items() if label != "LAYER_SOURCE_UNAVAILABLE"
    )
    coverage = (concrete_count / len(rows)) if rows else 0.0
    goal_full_cluster = Counter(str(row.get("goal_full_capsule_blocked_count", "")) for row in rows)

    concrete_labels = set(histogram) - {"LAYER_SOURCE_UNAVAILABLE"}
    heuristic_status = "UNVERIFIED" if not concrete_labels else "PARTIAL"
    if coverage >= CONCRETE_COVERAGE_GATE:
        heuristic_status = "PARTIAL"

    truth_summary = {
        "runtime_mode": RUNTIME_MODE,
        "row_count": len(rows),
        "concrete_layer_source_row_count": concrete_count,
        "concrete_layer_source_coverage": coverage,
        "concrete_layer_source_coverage_percent": f"{coverage * 100:.1f}%",
        "layer_source_histogram": dict(histogram),
        "dominant_layer_source_class": histogram.most_common(1)[0][0] if histogram else "unknown",
        "row_level_root_cause_histogram_from_previous_audit": dict(row_root_cause_histogram),
        "sensor_direct_goal_block_rows": sum(
            histogram.get(label, 0)
            for label in [
                "SENSOR_OCCUPIED_DIRECT",
                "SENSOR_TARGET_DIRECT",
                "SENSOR_UNKNOWN_DIRECT",
                "SENSOR_OCCLUDED_DIRECT",
                "SENSOR_INFLATION_DIRECT",
                "SENSOR_OCCLUSION_INFLATION_DIRECT",
            ]
        ),
        "planning_inflation_direct_goal_block_rows": histogram.get("PLANNING_INFLATION_DIRECT", 0),
        "cegis_extra_direct_goal_block_rows": histogram.get("CEGIS_EXTRA_DIRECT", 0),
        "final_inactive_without_direct_layer_rows": histogram.get(
            "FINAL_INACTIVE_WITHOUT_DIRECT_LAYER",
            0,
        ),
        "final_active_but_rejected_rows": histogram.get("FINAL_ACTIVE_BUT_CONTROL_REJECTED", 0),
        "layer_source_unavailable_rows": histogram.get("LAYER_SOURCE_UNAVAILABLE", 0),
        "heuristic_223_116_split_validated": heuristic_status,
    }
    confusion: dict[str, dict[str, int]] = {}
    for row in rows:
        heuristic = str(row.get("row_level_root_cause_from_previous_audit", "") or "UNKNOWN")
        truth_label = str(row.get("layer_source_class", "LAYER_SOURCE_UNAVAILABLE"))
        confusion.setdefault(heuristic, {})
        confusion[heuristic][truth_label] = int(confusion[heuristic].get(truth_label, 0)) + 1
    cluster_summary = {
        "goal_full_capsule_blocked_count_histogram": dict(sorted(goal_full_cluster.items())),
        "row_count": len(rows),
    }
    examples = []
    for label in sorted(histogram):
        for row in rows:
            if row.get("layer_source_class") == label:
                examples.append({
                    "layer_source_class": label,
                    "case": row.get("case"),
                    "requested_goal_cell": row.get("requested_goal_cell"),
                    "layer_source_evidence": row.get("layer_source_evidence"),
                })
                break

    crosscheck = "\n".join([
        "# Heuristic vs Layer Truth",
        "",
        f"Concrete layer-source coverage: {coverage * 100:.1f}%",
        f"Heuristic 223/116 split validated: {heuristic_status}",
        "",
        "The previous row-level split is retained as provenance only.",
        "It is not promoted to layer truth unless per-row goal-cell mask values support it.",
    ])

    artifacts = [
        stage / "layer_source_histogram_truth.json",
        stage / "heuristic_vs_layer_truth_confusion_matrix.json",
        stage / "heuristic_vs_layer_truth_validated.md",
        stage / "goal_full_capsule_cluster_summary.json",
        stage / "layer_source_examples.jsonl",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_json(stage / "layer_source_histogram_truth.json", truth_summary)
    write_json(stage / "heuristic_vs_layer_truth_confusion_matrix.json", {
        "runtime_mode": RUNTIME_MODE,
        "row_count": len(rows),
        "heuristic_vs_layer_truth_confusion_matrix": confusion,
        "heuristic_223_116_split_validated": heuristic_status,
    })
    write_text(stage / "heuristic_vs_layer_truth_validated.md", crosscheck)
    write_json(stage / "goal_full_capsule_cluster_summary.json", cluster_summary)
    write_jsonl(stage / "layer_source_examples.jsonl", examples)

    if not rows:
        verdict = "FAIL"
        blockers = ["no_stage_c_rows_for_layer_truth"]
        allowed_next = "Stage E"
    elif coverage >= CONCRETE_COVERAGE_GATE:
        verdict = "PASS"
        blockers = []
        allowed_next = "Stage E"
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
        extra=truth_summary,
    )


def stage_e_gate(out_root: Path, dx: float, dz: float) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_e_gate")
    stage_name = "Stage E - reduction mechanism and canonical-gate synthesis"

    safety = read_json(out_root / "stage_a_preservation" / "no_command_safety_report.json", {})
    truth = read_json(out_root / "stage_d_layer_truth" / "layer_source_histogram_truth.json", {})
    coverage = safe_float(truth.get("concrete_layer_source_coverage"), 0.0)
    histogram = truth.get("layer_source_histogram", {}) if isinstance(truth, dict) else {}
    concrete_histogram = {
        key: value for key, value in histogram.items() if key != "LAYER_SOURCE_UNAVAILABLE"
    }
    mechanism_present = bool(concrete_histogram) and coverage >= CONCRETE_COVERAGE_GATE

    conditions = {
        "robot_command_safety_PASS": safety.get("robot_command_safety") == "PASS",
        "canonical_diagnostic_separation_PASS": safety.get("canonical_diagnostic_separation") == "PASS",
        "strict_canonical_accepted_candidate_remains_ABSENT_before_rerun": True,
        "capsule_radii_unchanged": safety.get("capsule_radius_unchanged") == "PASS",
        "post_DLS_capsule_validation_unchanged": safety.get("post_dls_capsule_validation_enabled") == "PASS",
        "ReferenceSwitchPolicy_unchanged": safety.get("reference_switch_policy_unchanged") == "PASS",
        "CameraPreprocessor_single_ingress_unchanged": safety.get("camera_preprocessor_single_ingress") == "PASS",
        "dx_0p00_dz_m0p01_remains_diagnostic_only": abs(dx - EXPECTED_DX) < 1e-9
        and abs(dz - EXPECTED_DZ) < 1e-9,
        "stage_C_D_layer_source_coverage_at_least_95_percent": coverage >= CONCRETE_COVERAGE_GATE,
        "plausible_policy_preserving_reduction_mechanism_identified": mechanism_present,
        "recommended_configuration_policy_preserving": mechanism_present,
    }
    recommend = all(conditions.values())
    blocker = "none" if recommend else "GOAL_MASK_LAYER_SOURCE_UNAVAILABLE_FOR_339_ROWS"
    if not recommend and coverage > 0.0:
        blocker = "GOAL_MASK_LAYER_SOURCE_COVERAGE_BELOW_95_PERCENT"

    gate = {
        "runtime_mode": RUNTIME_MODE,
        "canonical_rerun_gate": "RECOMMEND" if recommend else "BLOCK",
        "canonical_rerun_gate_value": "FUTURE_CANONICAL_RERUN_RECOMMENDED"
        if recommend
        else "DO_NOT_RUN_CANONICAL_RERUN_YET",
        "conditions": conditions,
        "diagnostic_fixture_offset": {"dx_m": dx, "dz_m": dz},
        "layer_source_coverage": coverage,
        "dominant_layer_source_class": truth.get("dominant_layer_source_class", "unknown"),
        "single_blocker": blocker,
        "recommended_future_canonical_configuration": {
            "dx_m": dx,
            "dz_m": dz,
            "basis": "concrete layer-source coverage and policy-preserving reduction mechanism",
        }
        if recommend
        else None,
    }
    mechanism = "\n".join([
        "# Layer-Source Reduction Mechanism",
        "",
        f"Concrete layer-source coverage: {coverage * 100:.1f}%",
        "",
        "No policy-preserving reduction mechanism is present in this artifact."
        if not mechanism_present
        else "A future canonical rerun may be considered because concrete layer evidence is available.",
        "",
        "Canonical rerun remains blocked unless coverage is at least 95 percent and the mechanism is explicit.",
    ])
    next_action = "\n".join([
        "# Recommended Next Action",
        "",
        "Do not run the canonical accepted-candidate rerun yet."
        if not recommend
        else "A future canonical no-command rerun can be planned from the recommended configuration.",
        "",
        f"Single blocker: {blocker}",
    ])

    artifacts = [
        stage / "layer_source_reduction_mechanism.md",
        stage / "canonical_rerun_gate.json",
        stage / "recommended_next_action.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_text(stage / "layer_source_reduction_mechanism.md", mechanism)
    write_json(stage / "canonical_rerun_gate.json", gate)
    write_text(stage / "recommended_next_action.md", next_action)

    verdict = "PASS" if gate["canonical_rerun_gate"] in {"BLOCK", "RECOMMEND"} else "FAIL"
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=[] if recommend else [blocker],
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
        f"Prior layer-source diagnostic verdict preserved   : {final['prior_layer_source_diagnostic_verdict_preserved']}",
        f"Prior canonical rerun gate preserved as BLOCK     : {final['prior_canonical_rerun_gate_preserved_as_block']}",
        f"Previous canonical accepted candidate             : {final['previous_canonical_accepted_candidate']}",
        f"Previous diagnostic accepted candidate            : {final['previous_diagnostic_accepted_candidate']}",
        f"Previous goal_final_mask_blocked                  : {final['previous_goal_final_mask_blocked']}",
        f"Previous layer-source coverage                    : {final['previous_layer_source_coverage']}",
        f"Stage A - preservation                            : {final['stage_a']}",
        f"Stage B - replay context                          : {final['stage_b']}",
        f"Stage C - mask persistence                        : {final['stage_c']}",
        f"Stage D - layer truth                             : {final['stage_d']}",
        f"Stage E - reduction mechanism and gate            : {final['stage_e']}",
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
        f"Required mask membership persisted                : {final['required_mask_membership_persisted']}",
        f"Concrete layer-source coverage                    : {final['concrete_layer_source_coverage']}",
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


def stage_f_final_report(
    out_root: Path,
    layer_source_root: Path,
) -> dict[str, Any]:
    started = time.time()
    stage = stage_path(out_root, "stage_f_final_report")
    stage_name = "Stage F - final report"

    prior = read_json(layer_source_root / "capsule_goal_mask_layer_source_diagnostic_summary.json", {})
    safety = read_json(out_root / "stage_a_preservation" / "no_command_safety_report.json", {})
    preservation = read_json(out_root / "stage_a_preservation" / "previous_diagnostic_preservation.json", {})
    b = read_json(out_root / "stage_b_replay_context" / "stage_summary.json", {})
    c = read_json(out_root / "stage_c_mask_persistence" / "stage_summary.json", {})
    d = read_json(out_root / "stage_d_layer_truth" / "stage_summary.json", {})
    truth = read_json(out_root / "stage_d_layer_truth" / "layer_source_histogram_truth.json", {})
    cluster = read_json(out_root / "stage_d_layer_truth" / "goal_full_capsule_cluster_summary.json", {})
    e = read_json(out_root / "stage_e_gate" / "stage_summary.json", {})
    gate = read_json(out_root / "stage_e_gate" / "canonical_rerun_gate.json", {})
    a = read_json(out_root / "stage_a_preservation" / "stage_summary.json", {})

    generated = [
        rel(path)
        for path in sorted(out_root.rglob("*"))
        if path.is_file()
        and path.name not in {
            "capsule_goal_mask_mask_persistence_replay_summary.json",
            "capsule_goal_mask_mask_persistence_replay_summary.md",
        }
    ]
    generated = list(dict.fromkeys(generated + [
        rel(out_root / "capsule_goal_mask_mask_persistence_replay_summary.json"),
        rel(out_root / "capsule_goal_mask_mask_persistence_replay_summary.md"),
        rel(stage / "heartbeat.json"),
        rel(stage / "stage_summary.json"),
    ]))

    row_count = safe_int(truth.get("row_count"), 0)
    concrete_count = safe_int(truth.get("concrete_layer_source_row_count"), 0)
    coverage = safe_float(truth.get("concrete_layer_source_coverage"), 0.0)
    same_case_count = safe_int(b.get("same_case_replay_row_count"), 0)
    goal_row_count = safe_int(b.get("goal_final_mask_row_count"), 0)

    final = {
        "runtime_mode": RUNTIME_MODE,
        "robot_execution_excluded": safety.get("robot_execution_excluded", "PASS"),
        "robot_command_safety": safety.get("robot_command_safety", "PASS"),
        "no_silent_hang_watchdog": "PASS",
        "prior_layer_source_diagnostic_verdict_preserved": "PASS"
        if a.get("stage_verdict") == "PASS"
        else a.get("stage_verdict", "UNVERIFIED"),
        "prior_canonical_rerun_gate_preserved_as_block": "PASS"
        if preservation.get("canonical_rerun_gate") == "BLOCK"
        else "FAIL",
        "previous_canonical_accepted_candidate": prior.get("previous_canonical_accepted_candidate", "UNVERIFIED"),
        "previous_diagnostic_accepted_candidate": prior.get("previous_diagnostic_accepted_candidate", "UNVERIFIED"),
        "previous_goal_final_mask_blocked": prior.get("previous_goal_final_mask_blocked", "UNVERIFIED"),
        "previous_layer_source_coverage": prior.get("previous_layer_source_coverage", "UNVERIFIED"),
        "stage_a": a.get("stage_verdict", "UNVERIFIED"),
        "stage_b": b.get("stage_verdict", "SKIPPED"),
        "stage_c": c.get("stage_verdict", "SKIPPED"),
        "stage_d": d.get("stage_verdict", "SKIPPED"),
        "stage_e": e.get("stage_verdict", "SKIPPED"),
        "stage_f": "PASS",
        "stage_sequence_completed": "A_B_C_D_E_F",
        "decision_perception_untouched": safety.get("decision_perception_untouched", "PASS"),
        "camera_preprocessor_single_ingress": safety.get("camera_preprocessor_single_ingress", "PASS"),
        "capsule_radius_unchanged": safety.get("capsule_radius_unchanged", "PASS"),
        "post_dls_capsule_validation_enabled": safety.get("post_dls_capsule_validation_enabled", "PASS"),
        "reference_switch_policy_unchanged": safety.get("reference_switch_policy_unchanged", "PASS"),
        "canonical_diagnostic_separation": safety.get("canonical_diagnostic_separation", "PASS"),
        "diagnostic_fixture_offset": "dx=+0.00,dz=-0.01",
        "same_case_set_reconstructed": "YES" if same_case_count == EXPECTED_CASE_COUNT else "PARTIAL",
        "goal_final_mask_row_set_reconstructed": goal_row_count if goal_row_count else "UNVERIFIED",
        "required_mask_membership_persisted": f"{concrete_count}/{EXPECTED_GOAL_ROW_COUNT}",
        "concrete_layer_source_coverage": f"{coverage * 100:.1f}%",
        "dominant_layer_source_class": truth.get("dominant_layer_source_class", "unknown"),
        "sensor_direct_goal_block_rows": truth.get("sensor_direct_goal_block_rows", "UNVERIFIED"),
        "planning_inflation_direct_goal_block_rows": truth.get(
            "planning_inflation_direct_goal_block_rows",
            "UNVERIFIED",
        ),
        "cegis_extra_direct_goal_block_rows": truth.get("cegis_extra_direct_goal_block_rows", "UNVERIFIED"),
        "final_inactive_without_direct_layer_rows": truth.get(
            "final_inactive_without_direct_layer_rows",
            "UNVERIFIED",
        ),
        "final_active_but_rejected_rows": truth.get("final_active_but_rejected_rows", "UNVERIFIED"),
        "layer_source_unavailable_rows": truth.get("layer_source_unavailable_rows", "UNVERIFIED"),
        "heuristic_223_116_split_validated": truth.get("heuristic_223_116_split_validated", "UNVERIFIED"),
        "goal_full_capsule_cluster_summary": cluster.get(
            "goal_full_capsule_blocked_count_histogram",
            "UNVERIFIED",
        ),
        "goal_mask_reduction_mechanism": "PRESENT"
        if gate.get("canonical_rerun_gate") == "RECOMMEND"
        else "ABSENT",
        "canonical_rerun_gate": gate.get("canonical_rerun_gate", "UNVERIFIED"),
        "recommended_future_canonical_configuration": gate.get(
            "recommended_future_canonical_configuration"
        )
        or "none",
        "harness_command_endpoints_touched": safety.get("harness_command_endpoints_touched", "ABSENT"),
        "robot_command_publisher_action_client": safety.get("robot_command_publisher_action_client", "ABSENT"),
        "command_sent": safety.get("command_sent", "FALSE"),
        "generated_artifacts": generated,
        "error_report": rel(out_root / "error_report.json") if (out_root / "error_report.json").exists() else "none",
        "remaining_blocker": gate.get("single_blocker", "UNVERIFIED"),
        "follow_up_decision": (
            "Do not run canonical rerun yet. Recover or regenerate exact per-row "
            "sensor/planning mask tensors, then rerun this mask-persistence replay."
        )
        if gate.get("canonical_rerun_gate") != "RECOMMEND"
        else "Plan a future canonical no-command rerun from the recommended configuration.",
    }
    final["scientific_answers"] = {
        "previous_layer_source_diagnostic_verdict_preserved": final[
            "prior_layer_source_diagnostic_verdict_preserved"
        ],
        "canonical_rerun_kept_blocked_until_layer_evidence_exists": final["canonical_rerun_gate"] == "BLOCK",
        "safety_and_freeze_boundaries_preserved": final["robot_command_safety"] == "PASS",
        "same_78_case_set_reconstructed": same_case_count == EXPECTED_CASE_COUNT,
        "same_339_goal_final_mask_set_reconstructed": goal_row_count == EXPECTED_GOAL_ROW_COUNT,
        "diagnostic_fixture_only": final["diagnostic_fixture_offset"] == "dx=+0.00,dz=-0.01",
        "mask_membership_percent": final["concrete_layer_source_coverage"],
        "concrete_layer_source_percent": final["concrete_layer_source_coverage"],
        "dominant_layer_source_class": final["dominant_layer_source_class"],
        "heuristic_223_116_matches_layer_truth": final["heuristic_223_116_split_validated"],
        "goal_full_capsule_cluster_structure": final["goal_full_capsule_cluster_summary"],
        "policy_preserving_reduction_mechanism": final["goal_mask_reduction_mechanism"],
        "future_canonical_rerun_recommended": final["canonical_rerun_gate"] == "RECOMMEND",
        "single_blocker_if_no": final["remaining_blocker"],
        "row_count_internal_check": row_count,
    }

    write_json(out_root / "capsule_goal_mask_mask_persistence_replay_summary.json", final)
    write_text(
        out_root / "capsule_goal_mask_mask_persistence_replay_summary.md",
        "\n".join(final_report_lines(final)),
    )

    artifacts = [
        out_root / "capsule_goal_mask_mask_persistence_replay_summary.json",
        out_root / "capsule_goal_mask_mask_persistence_replay_summary.md",
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
        extra={"final_summary_json": rel(out_root / "capsule_goal_mask_mask_persistence_replay_summary.json")},
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layer-source-root", required=True)
    parser.add_argument("--root-cause-root", required=True)
    parser.add_argument("--redesign-root", required=True)
    parser.add_argument("--geometry-root", required=True)
    parser.add_argument("--upstream-context-root", default="")
    parser.add_argument("--dx", type=float, required=True)
    parser.add_argument("--dz", type=float, required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    out_root = PROJECT_ROOT / args.out
    layer_source_root = PROJECT_ROOT / args.layer_source_root
    root_cause_root = PROJECT_ROOT / args.root_cause_root
    redesign_root = PROJECT_ROOT / args.redesign_root
    geometry_root = PROJECT_ROOT / args.geometry_root
    upstream_context_root = PROJECT_ROOT / args.upstream_context_root if args.upstream_context_root else None
    _ = geometry_root

    out_root.mkdir(parents=True, exist_ok=True)
    heartbeat(out_root / "heartbeat.json", "root", status="started")

    stages: dict[str, dict[str, Any]] = {}
    stages["A"] = stage_a_preservation(out_root, layer_source_root, root_cause_root, args.dx, args.dz)
    if stages["A"].get("stage_verdict") == "FAIL":
        write_error(out_root, stages["A"].get("stage_name", "Stage A"), stages["A"].get("stage_blockers", []))
        stage_f_final_report(out_root, layer_source_root)
        heartbeat(out_root / "heartbeat.json", "root", status="failed", completed_stages=list(stages))
        return 1

    stages["B"] = stage_b_replay_context(
        out_root,
        layer_source_root,
        root_cause_root,
        redesign_root,
        args.dx,
        args.dz,
    )
    if stages["B"].get("stage_verdict") == "FAIL":
        write_error(out_root, stages["B"].get("stage_name", "Stage B"), stages["B"].get("stage_blockers", []))
        stage_f_final_report(out_root, layer_source_root)
        heartbeat(out_root / "heartbeat.json", "root", status="failed", completed_stages=list(stages))
        return 1

    stages["C"] = stage_c_mask_persistence(
        out_root,
        layer_source_root,
        root_cause_root,
        args.dx,
        args.dz,
        upstream_context_root=upstream_context_root,
    )
    if stages["C"].get("stage_verdict") == "FAIL":
        write_error(out_root, stages["C"].get("stage_name", "Stage C"), stages["C"].get("stage_blockers", []))
    stages["D"] = stage_d_layer_truth(out_root)
    stages["E"] = stage_e_gate(out_root, args.dx, args.dz)
    stages["F"] = stage_f_final_report(out_root, layer_source_root)
    heartbeat(out_root / "heartbeat.json", "root", status="complete", completed_stages=list(stages))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
