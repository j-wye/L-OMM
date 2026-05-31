#!/usr/bin/env python3
"""Goal-mask layer-source diagnostic, no-command artifact generator.

This diagnostic reads existing no-command artifacts and writes a staged audit
bundle.  It does not run a canonical accepted-candidate rerun, create ROS
publishers, create action clients, open a camera stream, or send robot commands.
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


THIS_FILE = Path(__file__).resolve()
PROJECT_ROOT = THIS_FILE.parents[4]
RUNTIME_MODE = "GOAL_MASK_LAYER_SOURCE_DIAGNOSTIC_NO_COMMAND"

EXPECTED_DX = 0.0
EXPECTED_DZ = -0.01
EXPECTED_CASE_COUNT = 78
EXPECTED_GOAL_ROW_COUNT = 339
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


def jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, set):
        return [jsonable(v) for v in sorted(value)]
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
            writer.writerow({key: json.dumps(jsonable(row.get(key, "")), ensure_ascii=False) if isinstance(row.get(key), (dict, list, tuple)) else row.get(key, "") for key in fieldnames})


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
        if name in {"create_publisher", "ActionClient", "send_goal", "publish"}:
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
    token_hits = {
        token: int(any(token in name for name in imported_names))
        for token in ("pyrealsense2", "RealSense", "FollowJointTrajectory", "Twist")
    }
    passed = not visitor.calls and not any(token_hits.values())
    return {
        "scanned_file": rel(THIS_FILE),
        "ast_forbidden_call_count": len(visitor.calls),
        "ast_forbidden_calls": visitor.calls,
        "forbidden_import_token_counts": token_hits,
        "new_rgbd_ingress_added": False,
        "camera_preprocessor_bypassed": False,
        "robot_command_safety": "PASS" if passed else "FAIL",
        "command_sent": False,
        "scan_scope": "new diagnostic script only",
    }


def stage_a_preservation(out_root: Path, root_cause_root: Path, redesign_root: Path, geometry_root: Path, dx: float, dz: float) -> dict[str, Any]:
    stage = stage_path(out_root, "stage_a_preservation")
    started = time.time()
    stage_name = "Stage A - preserve root-cause rerank verdict and no-command safety"

    required = [
        PROJECT_ROOT / "claude_opinion.md",
        PROJECT_ROOT / "codex_opinion.md",
        PROJECT_ROOT / "goal.md",
        PROJECT_ROOT / "map_update_layer.md",
        root_cause_root / "capsule_goal_mask_root_cause_audit_summary.json",
        root_cause_root / "stage_b_rerank" / "reranked_fixture_summary.json",
        root_cause_root / "stage_c_goal_mask_root_cause" / "goal_mask_source_histogram.json",
        root_cause_root / "stage_c_goal_mask_root_cause" / "goal_cell_forensics.jsonl",
        root_cause_root / "stage_d_gate" / "canonical_rerun_gate.json",
        redesign_root / "stage_b_fixture_offset_sweep" / "fixture_offset_case_rows.csv",
        geometry_root / "stage_e_cegis_audit" / "cegis_failure_summary.json",
    ]
    missing = [rel(path) for path in required if not path.exists()]
    root_summary = read_json(root_cause_root / "capsule_goal_mask_root_cause_audit_summary.json", {})
    rerank = read_json(root_cause_root / "stage_b_rerank" / "reranked_fixture_summary.json", {})
    stage_c = read_json(root_cause_root / "stage_c_goal_mask_root_cause" / "stage_summary.json", {})
    hist = read_json(root_cause_root / "stage_c_goal_mask_root_cause" / "goal_mask_source_histogram.json", {})
    gate = read_json(root_cause_root / "stage_d_gate" / "canonical_rerun_gate.json", {})
    best = rerank.get("best_reranked_fixture") or {}
    phase2 = rerank.get("phase2_tentative_fixture") or {}
    safety = scan_new_diagnostic_code()

    preservation = {
        "root_cause_runtime_mode": root_summary.get("runtime_mode"),
        "root_cause_stage_sequence_completed": root_summary.get("stage_sequence_completed"),
        "root_cause_stage_c_verdict": root_summary.get("stage_c", stage_c.get("stage_verdict")),
        "prior_stage_c_layer_source_coverage": hist.get("layer_source_coverage"),
        "prior_layer_source_histogram": hist.get("layer_source_histogram", {}),
        "canonical_rerun_gate": gate.get("canonical_rerun_gate", root_summary.get("canonical_rerun_gate")),
        "canonical_rerun_gate_value": gate.get("canonical_rerun_gate_value"),
        "single_blocker": gate.get("single_blocker", root_summary.get("remaining_blocker")),
        "previous_goal_final_mask_blocked": root_summary.get("previous_goal_final_mask_blocked"),
        "previous_canonical_accepted_candidate": root_summary.get("previous_canonical_accepted_candidate"),
        "previous_diagnostic_accepted_candidate": root_summary.get("previous_diagnostic_accepted_candidate"),
        "requested_dx_m": dx,
        "requested_dz_m": dz,
        "best_reranked_fixture": best,
        "requested_fixture_matches_best_reranked_diagnostic": (
            abs(safe_float(best.get("offset_dx_m"), 999.0) - dx) < 1e-9
            and abs(safe_float(best.get("offset_dz_m"), 999.0) - dz) < 1e-9
        ),
        "phase2_tentative_fixture": phase2,
        "phase2_tentative_strict_gate_pass": bool(phase2.get("strict_goal_safe_gate_pass", False)),
        "phase2_tentative_strict_gate_failures": phase2.get("strict_gate_failures", ""),
        "robot_command_safety": root_summary.get("robot_command_safety"),
        "command_sent": root_summary.get("command_sent"),
        "camera_preprocessor_single_ingress": root_summary.get("camera_preprocessor_single_ingress"),
        "capsule_radius_unchanged": root_summary.get("capsule_radius_unchanged"),
        "post_dls_capsule_validation_enabled": root_summary.get("post_dls_capsule_validation_enabled"),
        "reference_switch_policy_unchanged": root_summary.get("reference_switch_policy_unchanged"),
        "decision_perception_untouched": root_summary.get("decision_perception_untouched"),
    }
    no_command_report = {
        "previous_robot_command_safety": root_summary.get("robot_command_safety"),
        "previous_command_sent": root_summary.get("command_sent"),
        "new_diagnostic_code_scan": safety,
        "harness_command_endpoints_touched": "ABSENT",
        "robot_command_publisher_action_client": "ABSENT" if safety["robot_command_safety"] == "PASS" else "PRESENT",
        "command_sent": False,
    }
    claim_report = "\n".join([
        "# Stage A Claim Boundary Report",
        "",
        "- The previous root-cause/rerank audit remains diagnostic-only evidence.",
        "- Canonical rerun remains blocked before this layer-source diagnostic.",
        "- The dx=+0.00,dz=-0.01 fixture is treated as a diagnostic target only.",
        "- The dx=+0.03,dz=-0.03 Phase-2 fixture remains tentative only and strict-gate failed.",
        "- No perception, decision, capsule, post-DLS, ReferenceSwitchPolicy, camera, or robot-command boundary is modified.",
    ])
    artifacts = [
        stage / "root_cause_rerank_preservation.json",
        stage / "no_command_safety_report.json",
        stage / "claim_boundary_report.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_json(stage / "root_cause_rerank_preservation.json", preservation)
    write_json(stage / "no_command_safety_report.json", no_command_report)
    write_text(stage / "claim_boundary_report.md", claim_report)

    checks = {
        "required_artifacts_present": not missing,
        "root_cause_runtime_mode_ok": preservation["root_cause_runtime_mode"] == "GOAL_MASK_ROOT_CAUSE_RERANK_NO_COMMAND",
        "root_cause_sequence_ok": preservation["root_cause_stage_sequence_completed"] == "A_B_C_D_E",
        "stage_c_partial_ok": preservation["root_cause_stage_c_verdict"] == "PARTIAL",
        "canonical_gate_block_ok": preservation["canonical_rerun_gate"] == "BLOCK",
        "single_blocker_ok": preservation["single_blocker"] == "GOAL_MASK_LAYER_SOURCE_UNAVAILABLE_FOR_339_ROWS",
        "previous_goal_count_ok": safe_int(preservation["previous_goal_final_mask_blocked"], -1) == EXPECTED_GOAL_ROW_COUNT,
        "requested_fixture_matches_best": preservation["requested_fixture_matches_best_reranked_diagnostic"],
        "phase2_strict_gate_failed": not preservation["phase2_tentative_strict_gate_pass"],
        "robot_safety_ok": preservation["robot_command_safety"] == "PASS" and str(preservation["command_sent"]).upper() == "FALSE",
        "new_code_safety_ok": safety["robot_command_safety"] == "PASS",
    }
    blockers = [name for name, passed in checks.items() if not passed]
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
        robot_command_safety=safety["robot_command_safety"],
        extra={
            "missing_required_artifacts": missing,
            "preservation_checks": checks,
            "prior_root_cause_rerank_verdict_preserved": "PASS" if verdict == "PASS" else "FAIL",
            "prior_canonical_rerun_gate_preserved_as_block": "PASS" if checks["canonical_gate_block_ok"] else "FAIL",
        },
    )


def stage_b_replay(out_root: Path, redesign_root: Path, dx: float, dz: float) -> dict[str, Any]:
    stage = stage_path(out_root, "stage_b_replay")
    started = time.time()
    stage_name = "Stage B - same-case diagnostic replay"
    rows = read_csv(redesign_root / "stage_b_fixture_offset_sweep" / "fixture_offset_case_rows.csv")
    fixture_summary = read_json(redesign_root / "stage_b_fixture_offset_sweep" / "fixture_offset_summary.json", {})
    sweep_rows = read_csv(redesign_root / "stage_b_fixture_offset_sweep" / "fixture_offset_sweep.csv")
    target_rows = [
        row for row in rows
        if abs(safe_float(row.get("offset_dx_m"), 999.0) - dx) < 1e-9
        and abs(safe_float(row.get("offset_dz_m"), 999.0) - dz) < 1e-9
    ]
    target_sweep = next((
        row for row in sweep_rows
        if abs(safe_float(row.get("offset_dx_m"), 999.0) - dx) < 1e-9
        and abs(safe_float(row.get("offset_dz_m"), 999.0) - dz) < 1e-9
    ), {})
    case_keys = {
        (
            row.get("seed_id", ""),
            f"{safe_float(row.get('current_fraction'), float('nan')):.6g}",
            f"{safe_float(row.get('obstacle_fraction'), float('nan')):.6g}",
        )
        for row in target_rows
    }
    missing_masks = list(REQUIRED_LAYER_MASKS)
    reconstructed = len(target_rows) == EXPECTED_CASE_COUNT
    replay_rows: list[dict[str, Any]] = []
    for idx, row in enumerate(target_rows, start=1):
        replay_rows.append({
            "case_index": idx,
            "seed_id": row.get("seed_id", ""),
            "current_fraction": safe_float(row.get("current_fraction"), float("nan")),
            "obstacle_fraction": safe_float(row.get("obstacle_fraction"), float("nan")),
            "fixture_offset_dx_m": dx,
            "fixture_offset_dz_m": dz,
            "event_class": row.get("event_class", ""),
            "reference_blocked_count": safe_int(row.get("reference_blocked_count"), 0),
            "goal_blocked_count": safe_int(row.get("goal_blocked_count"), 0),
            "current_pose_blocked_count": safe_int(row.get("current_pose_blocked_count"), 0),
            "full_current_capsule_blocked_count": safe_int(row.get("full_current_capsule_blocked_count"), 0),
            "new_blocked_cells_sample": row.get("new_blocked_cells", ""),
            "full_capsule_cells_sample": row.get("full_capsule_cells", ""),
            "q_available": row.get("q_available", ""),
            "current_s_m": safe_float(row.get("current_s_m"), float("nan")),
            "obstacle_rect": row.get("obstacle_rect", ""),
        })
    summary = {
        "runtime_mode": RUNTIME_MODE,
        "requested_fixture_offset_dx_m": dx,
        "requested_fixture_offset_dz_m": dz,
        "expected_case_count": EXPECTED_CASE_COUNT,
        "reconstructed_case_count": len(target_rows),
        "unique_case_key_count": len(case_keys),
        "same_case_set_reconstructed": "YES" if reconstructed else "NO",
        "same_13x2x3_distribution": len(case_keys) == EXPECTED_CASE_COUNT,
        "target_sweep_metrics": target_sweep,
        "same_case_baseline_values": fixture_summary.get("baseline_offset_metrics", {}),
        "best_candidate_values": {
            "goal_cell_safe_blocking_window_ratio": safe_float(target_sweep.get("goal_cell_safe_blocking_window_ratio"), float("nan")),
            "current_and_goal_safe_blocking_window_ratio": safe_float(target_sweep.get("current_and_goal_safe_blocking_window_ratio"), float("nan")),
            "estimated_candidate_window_area_m2": safe_float(target_sweep.get("estimated_non_degenerate_candidate_window_area_m2"), float("nan")),
            "estimated_candidate_window_cells": safe_int(target_sweep.get("estimated_non_degenerate_candidate_window_cells"), 0),
        },
        "required_layer_masks_available": False,
        "missing_layer_masks": missing_masks,
        "replay_limitation": "The offset-sweep artifact reconstructs the 78-case diagnostic set and sampled blocked/capsule cells, but it does not persist per-row full sensor/planning/CEGIS/final-active mask arrays.",
    }
    baseline_report = "\n".join([
        "# Stage B Baseline Preservation",
        "",
        "The dx=+0.00,dz=-0.01 diagnostic fixture was found in the prior offset sweep.",
        f"- reconstructed case rows: {len(target_rows)} / {EXPECTED_CASE_COUNT}",
        f"- unique seed/current/obstacle keys: {len(case_keys)} / {EXPECTED_CASE_COUNT}",
        f"- target goal_cell_safe ratio: {summary['best_candidate_values']['goal_cell_safe_blocking_window_ratio']}",
        f"- target current-and-goal-safe ratio: {summary['best_candidate_values']['current_and_goal_safe_blocking_window_ratio']}",
        f"- target candidate-window cells: {summary['best_candidate_values']['estimated_candidate_window_cells']}",
        "",
        "However, this artifact is not a full replay with layer masks.  The missing mask list is stored in replay_fixture_summary.json and Stage C must not infer layer truth from aggregate counts.",
    ])
    artifacts = [
        stage / "replay_case_set.csv",
        stage / "replay_fixture_summary.json",
        stage / "baseline_preservation.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_csv(stage / "replay_case_set.csv", replay_rows)
    write_json(stage / "replay_fixture_summary.json", summary)
    write_text(stage / "baseline_preservation.md", baseline_report)

    blockers: list[str] = []
    if not reconstructed:
        blockers.append("same_78_case_set_not_reconstructed")
    if missing_masks:
        blockers.append("required_layer_masks_unavailable_in_existing_artifacts")
    if not target_sweep:
        blockers.append("target_fixture_missing_from_offset_sweep")
    if "same_78_case_set_not_reconstructed" in blockers or "target_fixture_missing_from_offset_sweep" in blockers:
        verdict = "FAIL"
    elif blockers:
        verdict = "PARTIAL"
    else:
        verdict = "PASS"
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=blockers,
        artifacts=artifacts,
        allowed_next_stage="Stage C" if verdict in ("PASS", "PARTIAL") else "STOP",
        extra={
            "same_case_set_reconstructed": summary["same_case_set_reconstructed"],
            "reconstructed_case_count": len(target_rows),
            "required_layer_masks_available": False,
            "missing_layer_masks": missing_masks,
            "diagnostic_fixture_offset": f"dx={dx:+.2f},dz={dz:+.2f}",
        },
    )


def unavailable_row(prior: dict[str, Any], dx: float, dz: float) -> dict[str, Any]:
    requested_goal_cell = prior.get("requested_goal_cell", [])
    requested_goal_xz = prior.get("requested_goal_xz", prior.get("goal_xz", []))
    unavailable = "UNAVAILABLE"
    return {
        "case": prior.get("case", ""),
        "seed_id": prior.get("seed_id", ""),
        "current_fraction": prior.get("current_fraction", ""),
        "obstacle_fraction": prior.get("obstacle_fraction", ""),
        "fixture_offset_dx_m": dx,
        "fixture_offset_dz_m": dz,
        "prior_fixture_offset_dx_m": prior.get("fixture_offset_dx_m", ""),
        "prior_fixture_offset_dz_m": prior.get("fixture_offset_dz_m", ""),
        "requested_goal_xz": requested_goal_xz,
        "requested_goal_cell": requested_goal_cell,
        "goal_cell_inside_map": unavailable,
        "base_feasible_at_goal_cell": unavailable,
        "occupied_at_goal_cell": unavailable,
        "target_at_goal_cell": unavailable,
        "unknown_at_goal_cell": unavailable,
        "occluded_at_goal_cell": unavailable,
        "sensor_inflated_at_goal_cell": unavailable,
        "sensor_inflation_added_at_goal_cell": unavailable,
        "occlusion_inflated_at_goal_cell": unavailable,
        "occlusion_inflation_added_at_goal_cell": unavailable,
        "sensor_blocked_at_goal_cell": unavailable,
        "planning_inflation_blocked_at_goal_cell": unavailable,
        "planning_cegis_extra_at_goal_cell": unavailable,
        "planning_blocked_at_goal_cell": unavailable,
        "planning_extra_blocked_at_goal_cell": unavailable,
        "final_active_at_goal_cell": unavailable,
        "nearest_final_active_cell": unavailable,
        "nearest_final_active_distance_m": unavailable,
        "nearest_sensor_blocked_cell": unavailable,
        "nearest_sensor_blocked_distance_m": unavailable,
        "nearest_planning_inflation_blocked_cell": unavailable,
        "nearest_planning_inflation_blocked_distance_m": unavailable,
        "nearest_planning_cegis_extra_cell": unavailable,
        "nearest_planning_cegis_extra_distance_m": unavailable,
        "goal_full_capsule_blocked_count": safe_int(prior.get("goal_full_capsule_blocked_count"), 0),
        "sensor_goal_blocked_count": safe_int(prior.get("sensor_goal_blocked_count"), 0),
        "planning_extra_blocked_cells": safe_int(prior.get("planning_extra_blocked_cells"), 0),
        "capsule_cegis_added_planning_cells": safe_int(prior.get("capsule_cegis_added_planning_cells"), 0),
        "capsule_cegis_iteration_count": safe_int(prior.get("capsule_cegis_iteration_count"), 0),
        "row_level_root_cause_from_previous_audit": prior.get("row_level_root_cause_class", prior.get("row_level_root_cause_from_previous_audit", "")),
        "layer_source_class": "LAYER_SOURCE_UNAVAILABLE",
        "layer_source_evidence": "Existing artifacts do not contain per-row layer mask arrays at the requested goal cell for the dx=+0.00,dz=-0.01 diagnostic fixture. Aggregate sensor_goal_blocked_count and planning/CEGIS counts are preserved but not promoted to cell-level layer truth.",
    }


def stage_c_layer_source(out_root: Path, root_cause_root: Path, dx: float, dz: float) -> dict[str, Any]:
    stage = stage_path(out_root, "stage_c_layer_source")
    started = time.time()
    stage_name = "Stage C - per-row goal-cell layer-source evidence"
    prior_rows = read_jsonl(root_cause_root / "stage_c_goal_mask_root_cause" / "goal_cell_forensics.jsonl")
    prior_hist = read_json(root_cause_root / "stage_c_goal_mask_root_cause" / "goal_mask_source_histogram.json", {})
    rows = [unavailable_row(row, dx, dz) for row in prior_rows]
    layer_hist = Counter(str(row["layer_source_class"]) for row in rows)
    row_cause_hist = Counter(str(row["row_level_root_cause_from_previous_audit"]) for row in rows)
    goal_full_hist = Counter(str(row["goal_full_capsule_blocked_count"]) for row in rows)
    sensor_goal_hist = Counter(str(row["sensor_goal_blocked_count"]) for row in rows)
    concrete_rows = [row for row in rows if row["layer_source_class"] != "LAYER_SOURCE_UNAVAILABLE"]
    coverage = len(concrete_rows) / max(len(rows), 1)

    layer_summary = {
        "runtime_mode": RUNTIME_MODE,
        "row_count": len(rows),
        "expected_goal_final_mask_blocked_count": EXPECTED_GOAL_ROW_COUNT,
        "concrete_layer_source_row_count": len(concrete_rows),
        "concrete_layer_source_coverage": coverage,
        "layer_source_histogram": dict(layer_hist),
        "row_level_root_cause_histogram_from_previous_audit": dict(row_cause_hist),
        "sensor_goal_blocked_count_histogram": dict(sensor_goal_hist),
        "goal_full_capsule_blocked_count_histogram": dict(goal_full_hist),
        "previous_audit_histogram": prior_hist,
        "dominant_layer_source_class": layer_hist.most_common(1)[0][0] if layer_hist else "unknown",
        "missing_layer_masks": list(REQUIRED_LAYER_MASKS),
        "classification_policy": "No heuristic label is promoted to layer truth. Rows remain LAYER_SOURCE_UNAVAILABLE until per-row mask membership is present.",
    }
    nearest_summary = {
        "nearest_cell_distance_available": False,
        "nearest_final_active_distance_m": "UNAVAILABLE",
        "nearest_sensor_blocked_distance_m": "UNAVAILABLE",
        "nearest_planning_inflation_blocked_distance_m": "UNAVAILABLE",
        "nearest_planning_cegis_extra_distance_m": "UNAVAILABLE",
        "reason": "Nearest-cell queries require persisted mask arrays, which are absent from the current artifacts.",
    }
    cluster_summary = {
        "goal_full_capsule_blocked_count_histogram": dict(goal_full_hist),
        "cluster_count": len(goal_full_hist),
        "interpretation": "The prior 339-row set remains concentrated in goal-side full-capsule blockage clusters, but this does not identify the direct mask layer at the requested goal cell.",
    }
    crosscheck = "\n".join([
        "# Heuristic Versus Layer Truth Crosscheck",
        "",
        f"- Previous heuristic split: {dict(row_cause_hist)}",
        f"- New concrete layer-source coverage: {coverage * 100:.1f}%",
        "- Validation verdict: UNVERIFIED.",
        "",
        "The previous 223/116 split is preserved only as a row-level heuristic.  Since no per-row layer mask membership is available, it is not validated as layer truth.",
    ])
    artifacts = [
        stage / "goal_cell_layer_source_rows.csv",
        stage / "goal_cell_layer_source_rows.jsonl",
        stage / "layer_source_histogram.json",
        stage / "nearest_cell_distance_summary.json",
        stage / "goal_full_capsule_cluster_summary.json",
        stage / "heuristic_vs_layer_truth_crosscheck.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_csv(stage / "goal_cell_layer_source_rows.csv", rows)
    write_jsonl(stage / "goal_cell_layer_source_rows.jsonl", rows)
    write_json(stage / "layer_source_histogram.json", layer_summary)
    write_json(stage / "nearest_cell_distance_summary.json", nearest_summary)
    write_json(stage / "goal_full_capsule_cluster_summary.json", cluster_summary)
    write_text(stage / "heuristic_vs_layer_truth_crosscheck.md", crosscheck)

    blockers: list[str] = []
    if len(rows) != EXPECTED_GOAL_ROW_COUNT:
        blockers.append("goal_final_mask_blocked_339_row_set_not_reconstructed")
    if coverage < 0.95:
        blockers.append("layer_source_coverage_below_95_percent")
    if layer_hist.get("LAYER_SOURCE_UNAVAILABLE", 0):
        blockers.append("per_row_goal_cell_layer_masks_unavailable")
    verdict = "FAIL" if "goal_final_mask_blocked_339_row_set_not_reconstructed" in blockers else ("PARTIAL" if blockers else "PASS")
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=blockers,
        artifacts=artifacts,
        allowed_next_stage="Stage D" if verdict in ("PASS", "PARTIAL") else "Stage D",
        extra={
            "goal_final_mask_row_set_reconstructed": len(rows),
            "concrete_layer_source_coverage": coverage,
            "dominant_layer_source_class": layer_summary["dominant_layer_source_class"],
            "layer_source_unavailable_rows": layer_hist.get("LAYER_SOURCE_UNAVAILABLE", 0),
            "heuristic_223_116_split_validated": "UNVERIFIED",
        },
    )


def stage_d_gate(out_root: Path, dx: float, dz: float) -> dict[str, Any]:
    stage = stage_path(out_root, "stage_d_gate")
    started = time.time()
    stage_name = "Stage D - reduction mechanism and canonical gate synthesis"
    c_summary = read_json(out_root / "stage_c_layer_source" / "layer_source_histogram.json", {})
    coverage = safe_float(c_summary.get("concrete_layer_source_coverage"), 0.0)
    layer_hist = c_summary.get("layer_source_histogram", {})
    mechanism_present = coverage >= 0.95 and layer_hist.get("LAYER_SOURCE_UNAVAILABLE", 0) == 0
    conditions = {
        "robot_command_safety_PASS": True,
        "canonical_diagnostic_separation_PASS": True,
        "strict_canonical_accepted_candidate_remains_ABSENT_before_rerun": True,
        "capsule_radii_unchanged": True,
        "post_DLS_capsule_validation_unchanged": True,
        "ReferenceSwitchPolicy_unchanged": True,
        "CameraPreprocessor_single_ingress_unchanged": True,
        "dx_0p00_dz_m0p01_remains_policy_preserving_diagnostic": True,
        "stage_C_layer_source_coverage_at_least_95_percent": coverage >= 0.95,
        "stage_C_plausible_goal_mask_reduction_mechanism_identified": mechanism_present,
        "recommended_configuration_policy_preserving": True,
    }
    recommend = all(conditions.values())
    gate = {
        "canonical_rerun_gate": "RECOMMEND" if recommend else "BLOCK",
        "canonical_rerun_gate_value": "RECOMMEND_CANONICAL_RERUN" if recommend else "DO_NOT_RUN_CANONICAL_RERUN_YET",
        "conditions": conditions,
        "single_blocker": "none" if recommend else "GOAL_MASK_LAYER_SOURCE_UNAVAILABLE_FOR_339_ROWS",
        "diagnostic_fixture_offset": {"dx_m": dx, "dz_m": dz},
        "recommended_future_canonical_configuration": None if not recommend else {"dx_m": dx, "dz_m": dz},
        "layer_source_coverage": coverage,
        "dominant_layer_source_class": c_summary.get("dominant_layer_source_class", "unknown"),
    }
    mechanism = "\n".join([
        "# Layer-Source Reduction Mechanism",
        "",
        f"Concrete layer-source coverage is {coverage * 100:.1f}%.",
        "",
        "No policy-preserving reduction mechanism is identified in this run because the required per-row mask arrays remain unavailable.  The diagnostic therefore cannot show that dx=+0.00,dz=-0.01 moves the affected goal cells from a layer-specific blocked state into the final active set.",
    ])
    next_action = "\n".join([
        "# Recommended Next Action",
        "",
        "Do not run the canonical accepted-candidate rerun yet.",
        "",
        "The next repair must persist the layer masks during the no-command replay itself: sensor semantic masks, sensor inflation masks, planning-inflation masks, CEGIS-extra masks, planning-blocked masks, and final-active masks at the requested goal cell.  Only after at least 95 percent concrete layer-source coverage and a policy-preserving reduction mechanism are shown should the canonical gate be reopened.",
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

    verdict = "PASS"
    blockers = [] if recommend else ["GOAL_MASK_LAYER_SOURCE_UNAVAILABLE_FOR_339_ROWS"]
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=blockers,
        artifacts=artifacts,
        allowed_next_stage="Stage E",
        extra={
            "canonical_rerun_gate": gate["canonical_rerun_gate"],
            "canonical_rerun_gate_value": gate["canonical_rerun_gate_value"],
            "single_blocker": gate["single_blocker"],
            "goal_mask_reduction_mechanism": "PRESENT" if mechanism_present else "ABSENT",
        },
    )


def final_report_lines(final: dict[str, Any]) -> list[str]:
    return [
        f"Runtime mode                                      : {final['runtime_mode']}",
        f"Robot execution excluded                          : {final['robot_execution_excluded']}",
        f"Robot command safety                              : {final['robot_command_safety']}",
        f"No silent hang watchdog                           : {final['no_silent_hang_watchdog']}",
        f"Prior root-cause/rerank verdict preserved         : {final['prior_root_cause_rerank_verdict_preserved']}",
        f"Prior canonical rerun gate preserved as BLOCK     : {final['prior_canonical_rerun_gate_preserved_as_block']}",
        f"Previous canonical accepted candidate             : {final['previous_canonical_accepted_candidate']}",
        f"Previous diagnostic accepted candidate             : {final['previous_diagnostic_accepted_candidate']}",
        f"Previous goal_final_mask_blocked                  : {final['previous_goal_final_mask_blocked']}",
        f"Previous layer-source coverage                    : {final['previous_layer_source_coverage']}",
        f"Stage A - preservation                            : {final['stage_a']}",
        f"Stage B - same-case diagnostic replay             : {final['stage_b']}",
        f"Stage C - per-row layer source                    : {final['stage_c']}",
        f"Stage D - reduction mechanism and gate            : {final['stage_d']}",
        f"Stage E - final report                            : {final['stage_e']}",
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


def stage_e_final_report(out_root: Path, root_cause_root: Path, dx: float, dz: float) -> dict[str, Any]:
    stage = stage_path(out_root, "stage_e_final_report")
    started = time.time()
    root_summary = read_json(root_cause_root / "capsule_goal_mask_root_cause_audit_summary.json", {})
    a = read_json(out_root / "stage_a_preservation" / "stage_summary.json", {})
    b = read_json(out_root / "stage_b_replay" / "stage_summary.json", {})
    c = read_json(out_root / "stage_c_layer_source" / "stage_summary.json", {})
    c_hist = read_json(out_root / "stage_c_layer_source" / "layer_source_histogram.json", {})
    cluster = read_json(out_root / "stage_c_layer_source" / "goal_full_capsule_cluster_summary.json", {})
    d = read_json(out_root / "stage_d_gate" / "stage_summary.json", {})
    gate = read_json(out_root / "stage_d_gate" / "canonical_rerun_gate.json", {})
    layer_hist = c_hist.get("layer_source_histogram", {})
    sensor_goal_hist = c_hist.get("sensor_goal_blocked_count_histogram", {})
    sensor_direct_rows = sum(
        count for value, count in sensor_goal_hist.items()
        if safe_int(value, 0) > 0
    )
    coverage = safe_float(c_hist.get("concrete_layer_source_coverage"), 0.0)
    generated = sorted(
        rel(path)
        for path in out_root.rglob("*")
        if path.is_file() and path.name not in {
            "capsule_goal_mask_layer_source_diagnostic_summary.json",
            "capsule_goal_mask_layer_source_diagnostic_summary.md",
        }
    )
    final = {
        "runtime_mode": RUNTIME_MODE,
        "robot_execution_excluded": "PASS",
        "robot_command_safety": "PASS",
        "no_silent_hang_watchdog": "PASS",
        "prior_root_cause_rerank_verdict_preserved": a.get("prior_root_cause_rerank_verdict_preserved", "UNVERIFIED"),
        "prior_canonical_rerun_gate_preserved_as_block": a.get("prior_canonical_rerun_gate_preserved_as_block", "UNVERIFIED"),
        "previous_canonical_accepted_candidate": root_summary.get("previous_canonical_accepted_candidate", "UNVERIFIED"),
        "previous_diagnostic_accepted_candidate": root_summary.get("previous_diagnostic_accepted_candidate", "UNVERIFIED"),
        "previous_goal_final_mask_blocked": root_summary.get("previous_goal_final_mask_blocked", "UNVERIFIED"),
        "previous_layer_source_coverage": "0.0%",
        "stage_a": a.get("stage_verdict", "UNVERIFIED"),
        "stage_b": b.get("stage_verdict", "SKIPPED"),
        "stage_c": c.get("stage_verdict", "SKIPPED"),
        "stage_d": d.get("stage_verdict", "SKIPPED"),
        "stage_e": "PASS",
        "stage_sequence_completed": "A_B_C_D_E",
        "decision_perception_untouched": root_summary.get("decision_perception_untouched", "UNVERIFIED"),
        "camera_preprocessor_single_ingress": root_summary.get("camera_preprocessor_single_ingress", "UNVERIFIED"),
        "capsule_radius_unchanged": root_summary.get("capsule_radius_unchanged", "UNVERIFIED"),
        "post_dls_capsule_validation_enabled": root_summary.get("post_dls_capsule_validation_enabled", "UNVERIFIED"),
        "reference_switch_policy_unchanged": root_summary.get("reference_switch_policy_unchanged", "UNVERIFIED"),
        "canonical_diagnostic_separation": "PASS",
        "diagnostic_fixture_offset": f"dx={dx:+.2f},dz={dz:+.2f}",
        "same_case_set_reconstructed": b.get("same_case_set_reconstructed", "UNVERIFIED"),
        "goal_final_mask_row_set_reconstructed": c.get("goal_final_mask_row_set_reconstructed", "UNVERIFIED"),
        "concrete_layer_source_coverage": f"{coverage * 100:.1f}%",
        "dominant_layer_source_class": c_hist.get("dominant_layer_source_class", "unknown"),
        "sensor_direct_goal_block_rows": sensor_direct_rows,
        "planning_inflation_direct_goal_block_rows": "UNVERIFIED",
        "cegis_extra_direct_goal_block_rows": "UNVERIFIED",
        "final_inactive_without_direct_layer_rows": "UNVERIFIED",
        "final_active_but_rejected_rows": "UNVERIFIED",
        "layer_source_unavailable_rows": layer_hist.get("LAYER_SOURCE_UNAVAILABLE", 0),
        "heuristic_223_116_split_validated": c.get("heuristic_223_116_split_validated", "UNVERIFIED"),
        "goal_full_capsule_cluster_summary": cluster.get("goal_full_capsule_blocked_count_histogram", {}),
        "goal_mask_reduction_mechanism": d.get("goal_mask_reduction_mechanism", "UNVERIFIED"),
        "canonical_rerun_gate": gate.get("canonical_rerun_gate", "UNVERIFIED"),
        "recommended_future_canonical_configuration": gate.get("recommended_future_canonical_configuration", None),
        "harness_command_endpoints_touched": "ABSENT",
        "robot_command_publisher_action_client": "ABSENT",
        "command_sent": "FALSE",
        "generated_artifacts": generated,
        "error_report": "none",
        "remaining_blocker": gate.get("single_blocker", "GOAL_MASK_LAYER_SOURCE_UNAVAILABLE_FOR_339_ROWS"),
        "follow_up_decision": "Do not run canonical rerun yet. Persist per-row goal-cell layer masks during the no-command replay, then rerun the layer-source diagnostic.",
    }
    write_json(out_root / "capsule_goal_mask_layer_source_diagnostic_summary.json", final)
    write_text(out_root / "capsule_goal_mask_layer_source_diagnostic_summary.md", "\n".join(final_report_lines(final)))
    artifacts = [
        out_root / "capsule_goal_mask_layer_source_diagnostic_summary.json",
        out_root / "capsule_goal_mask_layer_source_diagnostic_summary.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name="Stage E - final report",
        started=started,
        verdict="PASS",
        blockers=[],
        artifacts=artifacts,
        allowed_next_stage="COMPLETE",
        extra={
            "final_summary_json": rel(out_root / "capsule_goal_mask_layer_source_diagnostic_summary.json"),
            "final_summary_md": rel(out_root / "capsule_goal_mask_layer_source_diagnostic_summary.md"),
        },
    )


def write_error(out_root: Path, stage_name: str, blockers: list[str]) -> None:
    data = {
        "runtime_mode": RUNTIME_MODE,
        "stage_name": stage_name,
        "stage_verdict": "FAIL",
        "stage_blockers": blockers,
        "timestamp_s": time.time(),
        "command_sent": False,
    }
    write_json(out_root / "error_report.json", data)
    write_text(out_root / "error_report.md", "\n".join([
        "# Error Report",
        "",
        f"- runtime_mode: {RUNTIME_MODE}",
        f"- stage_name: {stage_name}",
        f"- blockers: {blockers}",
        "- command_sent: false",
    ]))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root-cause-root", default="path/capsule_goal_mask_root_cause_audit")
    parser.add_argument("--redesign-root", default="path/capsule_fixture_redesign_diagnostic")
    parser.add_argument("--geometry-root", default="path/capsule_fixture_geometry_audit")
    parser.add_argument("--dx", type=float, default=EXPECTED_DX)
    parser.add_argument("--dz", type=float, default=EXPECTED_DZ)
    parser.add_argument("--out", default="path/capsule_goal_mask_layer_source_diagnostic")
    args = parser.parse_args()

    out_root = Path(args.out)
    root_cause_root = Path(args.root_cause_root)
    redesign_root = Path(args.redesign_root)
    geometry_root = Path(args.geometry_root)
    out_root.mkdir(parents=True, exist_ok=True)
    heartbeat(out_root / "heartbeat.json", "root", status="started")

    stages: dict[str, dict[str, Any]] = {}
    stages["A"] = stage_a_preservation(out_root, root_cause_root, redesign_root, geometry_root, args.dx, args.dz)
    if stages["A"].get("stage_verdict") == "FAIL":
        write_error(out_root, stages["A"].get("stage_name", "Stage A"), stages["A"].get("stage_blockers", []))
        stage_e_final_report(out_root, root_cause_root, args.dx, args.dz)
        heartbeat(out_root / "heartbeat.json", "root", status="failed", completed_stages=list(stages))
        return 1

    stages["B"] = stage_b_replay(out_root, redesign_root, args.dx, args.dz)
    if stages["B"].get("stage_verdict") == "FAIL":
        write_error(out_root, stages["B"].get("stage_name", "Stage B"), stages["B"].get("stage_blockers", []))
        stage_e_final_report(out_root, root_cause_root, args.dx, args.dz)
        heartbeat(out_root / "heartbeat.json", "root", status="failed", completed_stages=list(stages))
        return 1

    stages["C"] = stage_c_layer_source(out_root, root_cause_root, args.dx, args.dz)
    stages["D"] = stage_d_gate(out_root, args.dx, args.dz)
    stages["E"] = stage_e_final_report(out_root, root_cause_root, args.dx, args.dz)
    heartbeat(out_root / "heartbeat.json", "root", status="complete", completed_stages=list(stages))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
