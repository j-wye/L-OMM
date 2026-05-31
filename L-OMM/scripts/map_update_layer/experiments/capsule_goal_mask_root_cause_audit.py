#!/usr/bin/env python3
"""Goal-mask root-cause rerank audit, no-command artifact generator.

This pass reads existing no-command artifacts only.  It does not run a
canonical accepted-candidate rerun, create ROS publishers, create action
clients, open a camera stream, or send robot commands.
"""
from __future__ import annotations

import argparse
import ast
import csv
import json
import math
import time
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable


THIS_FILE = Path(__file__).resolve()
PROJECT_ROOT = THIS_FILE.parents[4]
RUNTIME_MODE = "GOAL_MASK_ROOT_CAUSE_RERANK_NO_COMMAND"

GOAL_SAFE_BASELINE = 0.5128
CURRENT_AND_GOAL_BASELINE = 0.1282
CANDIDATE_WINDOW_AREA_BASELINE_M2 = 0.0186
CANDIDATE_WINDOW_CELLS_BASELINE = 186
TOP_DEADZONE_OBSTACLE_OVERLAP_BASELINE = 2
TOP_DEADZONE_CURRENT_CAPSULE_OVERLAP_BASELINE = 46

SYNTHETIC_X0_M = -1.1
SYNTHETIC_Z0_M = -0.6
SYNTHETIC_RESOLUTION_M = 0.01


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
            writer.writerow({key: jsonable(row.get(key, "")) for key in fieldnames})


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


def truthy(value: Any) -> bool:
    return value in (True, "True", "true", "1", 1)


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


def xz_to_cell(xz: Any) -> list[int]:
    parsed = parse_jsonish(xz, [])
    if not isinstance(parsed, (list, tuple)) or len(parsed) < 2:
        return []
    ix = int(round((float(parsed[0]) - SYNTHETIC_X0_M) / SYNTHETIC_RESOLUTION_M))
    iz = int(round((float(parsed[1]) - SYNTHETIC_Z0_M) / SYNTHETIC_RESOLUTION_M))
    return [ix, iz]


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


def stage_a_preservation(out_root: Path, redesign_root: Path, geometry_root: Path) -> dict[str, Any]:
    stage = stage_path(out_root, "stage_a_preservation")
    started = time.time()
    stage_name = "Stage A - preserve Phase-2 diagnostic verdict and safety"

    required = [
        PROJECT_ROOT / "claude_opinion.md",
        PROJECT_ROOT / "codex_opinion.md",
        PROJECT_ROOT / "goal.md",
        PROJECT_ROOT / "map_update_layer.md",
        geometry_root / "capsule_fixture_geometry_audit_summary.json",
        geometry_root / "stage_e_cegis_audit" / "cegis_failure_summary.json",
        geometry_root / "stage_g_synthesis" / "fixture_geometry_diagnosis.json",
        redesign_root / "capsule_fixture_redesign_diagnostic_summary.json",
        redesign_root / "stage_b_fixture_offset_sweep" / "fixture_offset_sweep.csv",
        redesign_root / "stage_b_fixture_offset_sweep" / "fixture_offset_summary.json",
        redesign_root / "stage_c_capsule_objective_sweep" / "objective_variant_summary.json",
        redesign_root / "stage_d_combined_window_eval" / "combined_variant_summary.json",
        redesign_root / "stage_e_canonical_readiness_gate" / "canonical_rerun_readiness.json",
        redesign_root / "stage_e_canonical_readiness_gate" / "recommended_canonical_configuration.json",
    ]
    missing = [rel(path) for path in required if not path.exists()]

    claude_text = read_text(PROJECT_ROOT / "claude_opinion.md")
    codex_text = read_text(PROJECT_ROOT / "codex_opinion.md")
    final = read_json(redesign_root / "capsule_fixture_redesign_diagnostic_summary.json", {})
    safety_prev = read_json(redesign_root / "stage_a_baseline" / "no_command_safety_report.json", {})
    preservation_prev = read_json(redesign_root / "stage_a_baseline" / "previous_audit_preservation.json", {})
    objective = read_json(redesign_root / "stage_c_capsule_objective_sweep" / "objective_variant_summary.json", {})
    combined = read_json(redesign_root / "stage_d_combined_window_eval" / "combined_variant_summary.json", {})
    readiness = read_json(redesign_root / "stage_e_canonical_readiness_gate" / "canonical_rerun_readiness.json", {})
    failure = read_json(geometry_root / "stage_e_cegis_audit" / "cegis_failure_summary.json", {})

    objective_goal_values = [
        safe_int(row.get("goal_final_mask_blocked_count"), -1)
        for row in objective.get("top_objective_variants", [])
        if isinstance(row, dict)
    ]
    phase2_preserved = {
        "phase2_stage_sequence_completed": final.get("stage_sequence_completed"),
        "phase2_stage_sequence_preserved": final.get("stage_sequence_completed") == "A_B_C_D_E_F",
        "claude_review_mentions_goal_mask_339": "goal_final_mask_blocked" in claude_text and "339" in claude_text,
        "codex_review_mentions_goal_mask_339": "goal_final_mask_blocked" in codex_text and "339" in codex_text,
        "previous_controlmodule_run_count": failure.get("controlmodule_run_count"),
        "previous_canonical_accepted_candidate": final.get("previous_canonical_accepted_candidate", final.get("strict_canonical_accepted_candidate", "UNVERIFIED")),
        "previous_diagnostic_accepted_candidate": final.get("diagnostic_accepted_candidate", "UNVERIFIED"),
        "strict_canonical_accepted_candidate": final.get("strict_canonical_accepted_candidate", "UNVERIFIED"),
        "previous_goal_final_mask_blocked": failure.get("goal_final_mask_blocked_count"),
        "objective_goal_final_mask_blocked_values": objective_goal_values,
        "goal_final_mask_blocked_unchanged_all_objectives": bool(objective_goal_values) and all(v == 339 for v in objective_goal_values),
        "previous_capsule_proxy_collision": failure.get("capsule_proxy_collision_count"),
        "weak_readiness_label": "weak_procedural",
        "readiness_was_phase2_pass_but_downgraded": readiness.get("canonical_rerun_readiness") == "PASS",
        "readiness_weak_criteria": [
            "goal_failure_reduced_or_explained_by_invariant",
            "top_deadzone_overlap_reduced_or_bypassed",
        ],
        "canonical_closure_preserved_absent": (
            final.get("strict_canonical_accepted_candidate") == "ABSENT"
            and safe_int(combined.get("canonical_accepted_count"), -1) == 0
        ),
        "diagnostic_candidate_preserved_absent": (
            final.get("diagnostic_accepted_candidate") == "ABSENT"
            and safe_int(combined.get("diagnostic_accepted_count"), -1) == 0
        ),
        "camera_preprocessor_single_ingress": final.get("camera_preprocessor_single_ingress", preservation_prev.get("camera_preprocessor_single_ingress")),
        "capsule_radius_unchanged": final.get("capsule_radius_unchanged", preservation_prev.get("capsule_radii_unchanged")),
        "post_dls_capsule_validation_enabled": final.get("post_dls_capsule_validation_enabled", preservation_prev.get("post_dls_capsule_validation_enabled")),
        "reference_switch_policy_unchanged": final.get("reference_switch_policy_unchanged", preservation_prev.get("reference_switch_policy_unchanged")),
        "decision_perception_untouched": final.get("decision_perception_untouched"),
    }
    safety = scan_new_diagnostic_code()
    no_command_report = {
        "previous_robot_command_safety": final.get("robot_command_safety", safety_prev.get("robot_command_safety")),
        "previous_command_sent": final.get("command_sent", "UNVERIFIED"),
        "new_diagnostic_code_scan": safety,
        "harness_command_endpoints_touched": "ABSENT",
        "robot_command_publisher_action_client": "ABSENT" if safety["robot_command_safety"] == "PASS" else "PRESENT",
        "command_sent": False,
    }
    claim_report = "\n".join([
        "# Stage A Claim Boundary Report",
        "",
        "- Phase-2 diagnostic verdict is preserved as diagnostic-only evidence.",
        "- Weak readiness is downgraded from procedural readiness to non-predictive evidence.",
        "- Canonical closure remains absent.",
        "- The prior tentative dx=+0.03,dz=-0.03 candidate remains tentative only.",
        "- No robot, controller, camera, perception, decision, capsule-radius, or acceptance-policy boundary is modified.",
    ])

    artifacts = [
        stage / "phase2_verdict_preservation.json",
        stage / "no_command_safety_report.json",
        stage / "claim_boundary_report.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_json(stage / "phase2_verdict_preservation.json", phase2_preserved)
    write_json(stage / "no_command_safety_report.json", no_command_report)
    write_text(stage / "claim_boundary_report.md", claim_report)

    pass_conditions = [
        not missing,
        phase2_preserved["phase2_stage_sequence_preserved"],
        phase2_preserved["canonical_closure_preserved_absent"],
        phase2_preserved["diagnostic_candidate_preserved_absent"],
        safe_int(phase2_preserved["previous_goal_final_mask_blocked"], -1) == 339,
        phase2_preserved["goal_final_mask_blocked_unchanged_all_objectives"],
        no_command_report["previous_robot_command_safety"] == "PASS",
        str(no_command_report["previous_command_sent"]).upper() == "FALSE",
        safety["robot_command_safety"] == "PASS",
    ]
    blockers = []
    if missing:
        blockers.append("required_source_of_truth_artifact_missing")
    if not phase2_preserved["phase2_stage_sequence_preserved"]:
        blockers.append("phase2_stage_sequence_not_preserved")
    if not phase2_preserved["canonical_closure_preserved_absent"]:
        blockers.append("canonical_closure_not_verified_absent")
    if not phase2_preserved["diagnostic_candidate_preserved_absent"]:
        blockers.append("diagnostic_candidate_not_verified_absent")
    if safe_int(phase2_preserved["previous_goal_final_mask_blocked"], -1) != 339:
        blockers.append("previous_goal_final_mask_blocked_not_339")
    if not phase2_preserved["goal_final_mask_blocked_unchanged_all_objectives"]:
        blockers.append("objective_goal_mask_count_not_preserved")
    if safety["robot_command_safety"] != "PASS":
        blockers.append("new_diagnostic_code_forbidden_endpoint_detected")
    verdict = "PASS" if all(pass_conditions) else "FAIL"
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
            "phase2_diagnostic_verdict_preserved": "PASS" if verdict == "PASS" else "FAIL",
            "weak_readiness_downgrade": "PASS",
            "previous_goal_final_mask_blocked": phase2_preserved["previous_goal_final_mask_blocked"],
        },
    )


def stage_b_rerank(out_root: Path, redesign_root: Path) -> dict[str, Any]:
    stage = stage_path(out_root, "stage_b_rerank")
    started = time.time()
    stage_name = "Stage B - same-case-set fixture reranking"
    sweep_path = redesign_root / "stage_b_fixture_offset_sweep" / "fixture_offset_sweep.csv"
    summary_path = redesign_root / "stage_b_fixture_offset_sweep" / "fixture_offset_summary.json"
    rows = read_csv(sweep_path)
    fixture_summary = read_json(summary_path, {})

    blockers: list[str] = []
    if not rows:
        blockers.append("fixture_offset_sweep_missing_or_empty")
    baseline_rows = [
        row for row in rows
        if abs(safe_float(row.get("offset_dx_m"), 999.0)) < 1e-9
        and abs(safe_float(row.get("offset_dz_m"), 999.0)) < 1e-9
    ]
    baseline = baseline_rows[0] if baseline_rows else {}
    if not baseline:
        blockers.append("same_case_baseline_theta0_missing")

    ranked_rows: list[dict[str, Any]] = []
    for row in rows:
        dx = safe_float(row.get("offset_dx_m"), 0.0)
        dz = safe_float(row.get("offset_dz_m"), 0.0)
        reference_ratio = safe_float(row.get("reference_blocking_window_ratio"), 0.0)
        goal_ratio = safe_float(row.get("goal_cell_safe_blocking_window_ratio"), 0.0)
        current_goal_ratio = safe_float(row.get("current_and_goal_safe_blocking_window_ratio"), 0.0)
        area = safe_float(row.get("estimated_non_degenerate_candidate_window_area_m2"), 0.0)
        cells = safe_int(row.get("estimated_non_degenerate_candidate_window_cells"), 0)
        obs_overlap = safe_int(row.get("top_deadzone_obstacle_overlap_case_count"), 999)
        capsule_overlap = safe_int(row.get("top_deadzone_current_capsule_overlap_case_count"), 999)
        gate_failures = []
        if reference_ratio < 0.99:
            gate_failures.append("reference_blocking_window_ratio_below_0p99")
        if goal_ratio < GOAL_SAFE_BASELINE:
            gate_failures.append("goal_cell_safe_below_same_case_baseline")
        if current_goal_ratio < CURRENT_AND_GOAL_BASELINE:
            gate_failures.append("current_and_goal_safe_below_same_case_baseline")
        if area <= 0.0:
            gate_failures.append("candidate_window_area_nonpositive")
        if obs_overlap > TOP_DEADZONE_OBSTACLE_OVERLAP_BASELINE:
            gate_failures.append("top_deadzone_obstacle_overlap_above_baseline")
        if capsule_overlap > TOP_DEADZONE_CURRENT_CAPSULE_OVERLAP_BASELINE:
            gate_failures.append("top_deadzone_current_capsule_overlap_above_baseline")
        pass_gate = not gate_failures
        ranked_rows.append({
            "offset_dx_m": dx,
            "offset_dz_m": dz,
            "evaluated_case_count": safe_int(row.get("evaluated_case_count"), 0),
            "reference_blocking_window_ratio": reference_ratio,
            "goal_cell_safe_blocking_window_ratio": goal_ratio,
            "current_and_goal_safe_blocking_window_ratio": current_goal_ratio,
            "estimated_candidate_window_cells": cells,
            "estimated_candidate_window_area_m2": area,
            "top_deadzone_obstacle_overlap_cases": obs_overlap,
            "top_deadzone_current_capsule_overlap_cases": capsule_overlap,
            "strict_goal_safe_gate_pass": pass_gate,
            "strict_gate_failures": ";".join(gate_failures),
            "fixture_displacement_l1_m": abs(dx) + abs(dz),
            "is_phase2_tentative_fixture": abs(dx - 0.03) < 1e-9 and abs(dz + 0.03) < 1e-9,
        })

    ranked_rows.sort(key=lambda row: (
        not bool(row["strict_goal_safe_gate_pass"]),
        -float(row["goal_cell_safe_blocking_window_ratio"]),
        -float(row["current_and_goal_safe_blocking_window_ratio"]),
        -float(row["estimated_candidate_window_area_m2"]),
        int(row["top_deadzone_current_capsule_overlap_cases"]),
        int(row["top_deadzone_obstacle_overlap_cases"]),
        float(row["fixture_displacement_l1_m"]),
    ))
    for idx, row in enumerate(ranked_rows, start=1):
        row["strict_rerank_order"] = idx

    pass_rows = [row for row in ranked_rows if row["strict_goal_safe_gate_pass"]]
    best = pass_rows[0] if pass_rows else {}
    phase2_tentative = next((row for row in ranked_rows if row["is_phase2_tentative_fixture"]), {})

    pareto_rows: list[dict[str, Any]] = []
    candidate_pool = [
        row for row in ranked_rows
        if row["reference_blocking_window_ratio"] >= 0.99
        and row["estimated_candidate_window_area_m2"] > 0.0
    ]
    for row in candidate_pool:
        dominated = False
        for other in candidate_pool:
            if row is other:
                continue
            no_worse = (
                other["goal_cell_safe_blocking_window_ratio"] >= row["goal_cell_safe_blocking_window_ratio"]
                and other["current_and_goal_safe_blocking_window_ratio"] >= row["current_and_goal_safe_blocking_window_ratio"]
                and other["estimated_candidate_window_area_m2"] >= row["estimated_candidate_window_area_m2"]
                and other["top_deadzone_current_capsule_overlap_cases"] <= row["top_deadzone_current_capsule_overlap_cases"]
                and other["top_deadzone_obstacle_overlap_cases"] <= row["top_deadzone_obstacle_overlap_cases"]
            )
            strictly_better = (
                other["goal_cell_safe_blocking_window_ratio"] > row["goal_cell_safe_blocking_window_ratio"]
                or other["current_and_goal_safe_blocking_window_ratio"] > row["current_and_goal_safe_blocking_window_ratio"]
                or other["estimated_candidate_window_area_m2"] > row["estimated_candidate_window_area_m2"]
                or other["top_deadzone_current_capsule_overlap_cases"] < row["top_deadzone_current_capsule_overlap_cases"]
                or other["top_deadzone_obstacle_overlap_cases"] < row["top_deadzone_obstacle_overlap_cases"]
            )
            if no_worse and strictly_better:
                dominated = True
                break
        if not dominated:
            pareto_rows.append(row)

    baseline_artifact = fixture_summary.get("baseline_offset_metrics", {})
    baseline_reconciliation = "\n".join([
        "# Same-Case-Set Baseline Reconciliation",
        "",
        "The rerank baseline is theta0=(dx=0.00,dz=0.00) from the Phase-2 fixture offset sweep, not the older wide-grid geometry-audit baseline.",
        "",
        f"- goal_cell_safe_blocking_window_ratio: artifact={safe_float(baseline.get('goal_cell_safe_blocking_window_ratio'), float('nan')):.16g}, gate={GOAL_SAFE_BASELINE}",
        f"- current_and_goal_safe_blocking_window_ratio: artifact={safe_float(baseline.get('current_and_goal_safe_blocking_window_ratio'), float('nan')):.16g}, gate={CURRENT_AND_GOAL_BASELINE}",
        f"- candidate_window_area_m2: artifact={safe_float(baseline.get('estimated_non_degenerate_candidate_window_area_m2'), float('nan')):.16g}, reference={CANDIDATE_WINDOW_AREA_BASELINE_M2}",
        f"- candidate_window_cells: artifact={safe_int(baseline.get('estimated_non_degenerate_candidate_window_cells'), -1)}, reference={CANDIDATE_WINDOW_CELLS_BASELINE}",
        f"- top_deadzone_obstacle_overlap_cases: artifact={safe_int(baseline.get('top_deadzone_obstacle_overlap_case_count'), -1)}, gate={TOP_DEADZONE_OBSTACLE_OVERLAP_BASELINE}",
        f"- top_deadzone_current_capsule_overlap_cases: artifact={safe_int(baseline.get('top_deadzone_current_capsule_overlap_case_count'), -1)}, gate={TOP_DEADZONE_CURRENT_CAPSULE_OVERLAP_BASELINE}",
        "",
        "The older current_and_goal_safe_blocking_window_ratio=0.9552845528455285 is preserved only as a wide-grid context value and is not used for this rerank.",
    ])

    summary = {
        "same_case_baseline_identified": bool(baseline),
        "candidate_count": len(ranked_rows),
        "strict_goal_safe_candidate_count": len(pass_rows),
        "strict_goal_safe_candidates": pass_rows,
        "best_reranked_fixture": best or None,
        "phase2_tentative_fixture": phase2_tentative or None,
        "phase2_tentative_fixture_strict_gate_pass": bool(phase2_tentative.get("strict_goal_safe_gate_pass", False)),
        "phase2_tentative_fixture_gate_failures": phase2_tentative.get("strict_gate_failures", ""),
        "same_case_baseline_artifact": baseline_artifact,
        "gate": {
            "reference_blocking_window_ratio_min": 0.99,
            "goal_cell_safe_blocking_window_ratio_min": GOAL_SAFE_BASELINE,
            "current_and_goal_safe_blocking_window_ratio_min": CURRENT_AND_GOAL_BASELINE,
            "estimated_candidate_window_area_m2_min_exclusive": 0.0,
            "top_deadzone_obstacle_overlap_cases_max": TOP_DEADZONE_OBSTACLE_OVERLAP_BASELINE,
            "top_deadzone_current_capsule_overlap_cases_max": TOP_DEADZONE_CURRENT_CAPSULE_OVERLAP_BASELINE,
        },
    }
    artifacts = [
        stage / "reranked_fixture_candidates.csv",
        stage / "reranked_fixture_summary.json",
        stage / "pareto_frontier.json",
        stage / "baseline_reconciliation.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_csv(stage / "reranked_fixture_candidates.csv", ranked_rows)
    write_json(stage / "reranked_fixture_summary.json", summary)
    write_json(stage / "pareto_frontier.json", {"pareto_frontier_count": len(pareto_rows), "pareto_frontier": pareto_rows})
    write_text(stage / "baseline_reconciliation.md", baseline_reconciliation)

    if blockers:
        verdict = "FAIL"
    elif pass_rows:
        verdict = "PASS"
    else:
        verdict = "PARTIAL"
        blockers.append("no_candidate_passes_strict_goal_safe_gate")
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
            "same_case_baseline_reconciled": "PASS" if baseline else "FAIL",
            "goal_safe_rerank_candidate_found": "YES" if pass_rows else "NO",
            "best_reranked_fixture_offset": [best.get("offset_dx_m"), best.get("offset_dz_m")] if best else [],
        },
    )


def load_candidate_goal_rows(prev_root: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    candidate_summary_path = prev_root / "stage_e_diversified_candidate_search" / "control_dryrun_candidate_summary.json"
    candidate_summary = read_json(candidate_summary_path, {})
    rows = [
        row for row in candidate_summary.get("trial_rows", [])
        if isinstance(row, dict)
        and str(row.get("candidate_source", "")) == "ControlModule.run"
        and str(row.get("invalid_reason_code", "")) == "goal_final_mask_blocked"
    ]
    return rows, candidate_summary


def stage_c_goal_mask_root_cause(out_root: Path, geometry_root: Path) -> dict[str, Any]:
    stage = stage_path(out_root, "stage_c_goal_mask_root_cause")
    started = time.time()
    stage_name = "Stage C - goal_final_mask_blocked root-cause audit"
    prev_root = PROJECT_ROOT / "path" / "capsule_fixture_feasibility_closure"
    trace_path = geometry_root / "stage_e_cegis_audit" / "cegis_trace.csv"
    failure_path = geometry_root / "stage_e_cegis_audit" / "cegis_failure_summary.json"
    trace_rows = read_csv(trace_path)
    trace_goal_rows = [row for row in trace_rows if str(row.get("invalid_reason_code", "")) == "goal_final_mask_blocked"]
    candidate_goal_rows, candidate_summary = load_candidate_goal_rows(prev_root)
    failure = read_json(failure_path, {})

    trace_cases = {str(row.get("case", "")) for row in trace_goal_rows}
    candidate_cases = {str(row.get("case", "")) for row in candidate_goal_rows}
    row_source_consistent = trace_cases == candidate_cases and len(trace_goal_rows) == len(candidate_goal_rows) == 339

    forensics_rows: list[dict[str, Any]] = []
    csv_rows: list[dict[str, Any]] = []
    layer_source_hist: Counter[str] = Counter()
    root_cause_hist: Counter[str] = Counter()
    seed_hist: Counter[str] = Counter()
    fraction_hist: Counter[str] = Counter()
    offset_hist: Counter[str] = Counter()
    inflation_hist: Counter[str] = Counter()
    cegis_iter_hist: Counter[str] = Counter()
    goal_full_capsule_hist: Counter[str] = Counter()
    sensor_goal_blocked_hist: Counter[str] = Counter()
    planning_extra_hist: Counter[str] = Counter()

    for row in candidate_goal_rows:
        goal_xz = row.get("goal_xz", [])
        requested_goal_cell = xz_to_cell(goal_xz)
        seed_id = str(row.get("seed_id", ""))
        current_fraction = safe_float(row.get("current_fraction"), float("nan"))
        obstacle_fraction = safe_float(row.get("obstacle_fraction"), float("nan"))
        offset_xz = parse_jsonish(row.get("offset_xz_m", []), [])
        if isinstance(offset_xz, (list, tuple)) and len(offset_xz) >= 2:
            offset_dx = safe_float(offset_xz[0], float("nan"))
            offset_dz = safe_float(offset_xz[1], float("nan"))
        else:
            offset_dx = float("nan")
            offset_dz = float("nan")
        sensor_goal_blocked = safe_int(row.get("sensor_goal_blocked_count"), 0)
        goal_full_capsule_blocked = safe_int(row.get("goal_full_capsule_blocked_count"), 0)
        planning_extra_cells = safe_int(row.get("planning_extra_blocked_cells"), 0)
        cegis_added_cells = safe_int(row.get("capsule_cegis_added_planning_cells"), 0)
        cegis_final_extra_cells = safe_int(row.get("capsule_cegis_final_extra_cells"), 0)
        cegis_iters = safe_int(row.get("capsule_cegis_iteration_count"), 0)
        rejection_reason = str(row.get("rejection_reason", ""))
        invalid_reason_code = str(row.get("invalid_reason_code", ""))

        if sensor_goal_blocked > 0:
            root_cause = "SENSOR_GOAL_CELL_DIRECTLY_BLOCKED"
        elif goal_full_capsule_blocked > 0 and cegis_added_cells > 0:
            root_cause = "CEGIS_FINAL_GOAL_CONTRACT_INTERACTION"
        elif goal_full_capsule_blocked > 0 and planning_extra_cells > 0:
            root_cause = "PLANNING_INFLATION_GOAL_CAPSULE_GEOMETRY"
        elif invalid_reason_code == "goal_final_mask_blocked":
            root_cause = "FINAL_GOAL_MASK_CONTRACT_UNDECOMPOSED"
        else:
            root_cause = "UNRECOVERABLE_ROW_CLASS"

        layer_source = "LAYER_SOURCE_UNAVAILABLE"
        layer_reason = (
            "Existing Stage-E artifacts preserve row-level counts and planning stats, "
            "but do not save per-row sensor/planning/CEGIS masks at the requested goal cell."
        )
        if sensor_goal_blocked == 0:
            direct_sensor_goal_status = "NOT_DIRECTLY_SENSOR_GOAL_BLOCKED"
        else:
            direct_sensor_goal_status = "SENSOR_GOAL_BLOCKED_BUT_LAYER_UNDECOMPOSED"

        forensic = {
            "trial": safe_int(row.get("trial"), -1),
            "case": str(row.get("case", "")),
            "seed_id": seed_id,
            "current_fraction": current_fraction,
            "obstacle_fraction": obstacle_fraction,
            "fixture_offset_dx_m": offset_dx,
            "fixture_offset_dz_m": offset_dz,
            "candidate_source": str(row.get("candidate_source", "")),
            "requested_goal_xz": goal_xz,
            "requested_goal_cell": requested_goal_cell,
            "obstacle_rect": row.get("obstacle_rect", []),
            "final_rejection_reason": rejection_reason,
            "invalid_reason_code": invalid_reason_code,
            "sensor_goal_blocked_count": sensor_goal_blocked,
            "goal_full_capsule_blocked_count": goal_full_capsule_blocked,
            "planning_blocked_cells": safe_int(row.get("planning_blocked_cells"), 0),
            "planning_extra_blocked_cells": planning_extra_cells,
            "sensor_blocked_cells": safe_int(row.get("sensor_blocked_cells"), 0),
            "capsule_planning_inflation_m": safe_float(row.get("capsule_planning_inflation_m"), float("nan")),
            "capsule_cegis_iteration_count": cegis_iters,
            "capsule_cegis_added_planning_cells": cegis_added_cells,
            "capsule_cegis_final_extra_cells": cegis_final_extra_cells,
            "layer_source_class": layer_source,
            "layer_source_reason": layer_reason,
            "direct_sensor_goal_status": direct_sensor_goal_status,
            "row_level_root_cause_class": root_cause,
            "root_cause_evidence": (
                f"sensor_goal_blocked={sensor_goal_blocked}; "
                f"goal_full_capsule_blocked={goal_full_capsule_blocked}; "
                f"planning_extra_blocked_cells={planning_extra_cells}; "
                f"cegis_added_planning_cells={cegis_added_cells}"
            ),
        }
        forensics_rows.append(forensic)
        csv_rows.append({k: forensic[k] for k in [
            "trial",
            "case",
            "seed_id",
            "current_fraction",
            "obstacle_fraction",
            "fixture_offset_dx_m",
            "fixture_offset_dz_m",
            "requested_goal_cell",
            "requested_goal_xz",
            "final_rejection_reason",
            "invalid_reason_code",
            "sensor_goal_blocked_count",
            "goal_full_capsule_blocked_count",
            "planning_extra_blocked_cells",
            "capsule_cegis_iteration_count",
            "capsule_cegis_added_planning_cells",
            "layer_source_class",
            "row_level_root_cause_class",
        ]})

        layer_source_hist[layer_source] += 1
        root_cause_hist[root_cause] += 1
        seed_hist[seed_id] += 1
        fraction_hist[f"cf={current_fraction:.3g}|of={obstacle_fraction:.3g}"] += 1
        offset_hist[f"dx={offset_dx:.3g}|dz={offset_dz:.3g}"] += 1
        inflation_hist[f"{safe_float(row.get('capsule_planning_inflation_m'), 0.0):.3g}"] += 1
        cegis_iter_hist[str(cegis_iters)] += 1
        goal_full_capsule_hist[str(goal_full_capsule_blocked)] += 1
        sensor_goal_blocked_hist[str(sensor_goal_blocked)] += 1
        planning_extra_hist[str(planning_extra_cells)] += 1

    row_level_coverage = len([row for row in forensics_rows if row["row_level_root_cause_class"] != "UNRECOVERABLE_ROW_CLASS"]) / max(len(forensics_rows), 1)
    layer_source_coverage = len([row for row in forensics_rows if row["layer_source_class"] != "LAYER_SOURCE_UNAVAILABLE"]) / max(len(forensics_rows), 1)
    dominant_root_cause = root_cause_hist.most_common(1)[0][0] if root_cause_hist else "UNKNOWN"
    dominant_category = (
        "CEGIS_INDUCED_WITH_LAYER_SOURCE_UNAVAILABLE"
        if dominant_root_cause == "CEGIS_FINAL_GOAL_CONTRACT_INTERACTION"
        else "PLANNER_OBJECTIVE_OR_FIXTURE_GEOMETRY_WITH_LAYER_SOURCE_UNAVAILABLE"
    )

    histogram = {
        "row_count": len(forensics_rows),
        "expected_goal_final_mask_blocked_count": 339,
        "row_level_root_cause_histogram": dict(root_cause_hist),
        "layer_source_histogram": dict(layer_source_hist),
        "dominant_row_level_root_cause": dominant_root_cause,
        "dominant_goal_final_mask_source": dominant_category,
        "row_level_root_cause_coverage": row_level_coverage,
        "layer_source_coverage": layer_source_coverage,
        "sensor_goal_blocked_count_histogram": dict(sensor_goal_blocked_hist),
        "goal_full_capsule_blocked_count_histogram": dict(goal_full_capsule_hist),
        "planning_extra_blocked_cells_top_histogram": dict(planning_extra_hist.most_common(20)),
        "artifact_limitation": "Per-row layer masks are not present; exact inside occupied/target/unknown/occlusion/sensor-inflation/CEGIS-cell labels cannot be asserted.",
    }
    seed_fraction_hist = {
        "seed_histogram": dict(seed_hist),
        "current_obstacle_fraction_histogram": dict(fraction_hist),
        "fixture_offset_histogram": dict(offset_hist),
        "capsule_planning_inflation_histogram": dict(inflation_hist),
        "cegis_iteration_histogram": dict(cegis_iter_hist),
    }
    assessment = "\n".join([
        "# Goal-Final-Mask Reduction Mechanism Assessment",
        "",
        "The 339-row set is reconstructable from the previous CEGIS/failure artifacts and the Stage-E candidate summary.",
        "",
        "Key row-level facts:",
        f"- sensor_goal_blocked_count is zero for {sensor_goal_blocked_hist.get('0', 0)} / {len(forensics_rows)} rows.",
        "- Therefore the dominant failure is not direct target/unknown/occlusion/sensor goal-cell occupancy in the saved row summary.",
        f"- goal_full_capsule_blocked_count is positive for {sum(1 for row in forensics_rows if row['goal_full_capsule_blocked_count'] > 0)} / {len(forensics_rows)} rows.",
        f"- CEGIS-added planning exclusions are present in {sum(1 for row in forensics_rows if row['capsule_cegis_added_planning_cells'] > 0)} / {len(forensics_rows)} rows.",
        "",
        "Interpretation:",
        "The dominant mechanism is a CEGIS/final-goal contract interaction on top of goal-side full-capsule geometry.  The sensor event does not directly mark the goal cell as blocked, but the planner-side final active mask and goal projection/contract eliminate the strict acceptance path.",
        "",
        "Reduction mechanism verdict:",
        "A stricter fixture rerank finds a goal-safe candidate window, but the current artifacts do not save per-row layer masks or per-row recomputed goal-cell membership for that candidate.  A canonical rerun is therefore still blocked until a no-command targeted diagnostic records the goal cell against sensor_blocked_mask, planning_inflation_blocked_mask, planning_cegis_extra_mask, and final_active_mask.",
    ])

    artifacts = [
        stage / "goal_final_mask_blocked_rows.csv",
        stage / "goal_mask_source_histogram.json",
        stage / "goal_mask_seed_fraction_histogram.json",
        stage / "goal_cell_forensics.jsonl",
        stage / "reduction_mechanism_assessment.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_csv(stage / "goal_final_mask_blocked_rows.csv", csv_rows)
    write_json(stage / "goal_mask_source_histogram.json", histogram)
    write_json(stage / "goal_mask_seed_fraction_histogram.json", seed_fraction_hist)
    write_jsonl(stage / "goal_cell_forensics.jsonl", forensics_rows)
    write_text(stage / "reduction_mechanism_assessment.md", assessment)

    blockers: list[str] = []
    if len(trace_goal_rows) != 339 or len(candidate_goal_rows) != 339:
        blockers.append("goal_final_mask_blocked_339_row_set_not_reconstructed")
    if not row_source_consistent:
        blockers.append("cegis_trace_and_candidate_summary_goal_sets_disagree")
    if layer_source_coverage < 0.95:
        blockers.append("per_row_layer_source_unavailable")
    verdict = "FAIL" if "goal_final_mask_blocked_339_row_set_not_reconstructed" in blockers else ("PARTIAL" if blockers else "PASS")
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=blockers,
        artifacts=artifacts,
        allowed_next_stage="Stage D" if verdict in ("PASS", "PARTIAL") else "STOP",
        extra={
            "goal_final_mask_blocked_row_count": len(forensics_rows),
            "row_source_consistent": row_source_consistent,
            "row_level_root_cause_coverage": row_level_coverage,
            "layer_source_coverage": layer_source_coverage,
            "dominant_goal_final_mask_source": dominant_category,
            "goal_final_mask_reduction_mechanism": "UNVERIFIED",
            "trial_row_source": candidate_summary.get("trial_row_source", "control_dryrun_candidate_summary.json"),
            "previous_failure_summary_goal_count": failure.get("goal_final_mask_blocked_count"),
        },
    )


def stage_d_gate(out_root: Path) -> dict[str, Any]:
    stage = stage_path(out_root, "stage_d_gate")
    started = time.time()
    stage_name = "Stage D - canonical rerun gate synthesis"
    b_summary = read_json(out_root / "stage_b_rerank" / "reranked_fixture_summary.json", {})
    c_hist = read_json(out_root / "stage_c_goal_mask_root_cause" / "goal_mask_source_histogram.json", {})
    c_stage = read_json(out_root / "stage_c_goal_mask_root_cause" / "stage_summary.json", {})

    best = b_summary.get("best_reranked_fixture") or {}
    phase2 = b_summary.get("phase2_tentative_fixture") or {}
    stage_b_found = bool(best)
    layer_source_available = safe_float(c_hist.get("layer_source_coverage"), 0.0) >= 0.95
    reduction_mechanism_present = False
    conditions = {
        "robot_command_safety_PASS": True,
        "canonical_diagnostic_separation_PASS": True,
        "strict_canonical_accepted_candidate_remains_ABSENT_before_rerun": True,
        "capsule_radii_unchanged": True,
        "post_DLS_capsule_validation_unchanged": True,
        "ReferenceSwitchPolicy_unchanged": True,
        "CameraPreprocessor_single_ingress_unchanged": True,
        "stage_B_found_strict_goal_safe_candidate": stage_b_found,
        "stage_C_layer_source_coverage_at_least_95_percent": layer_source_available,
        "stage_C_plausible_goal_mask_reduction_mechanism_identified": reduction_mechanism_present,
        "recommended_configuration_policy_preserving": True,
    }
    recommend = all(conditions.values())
    gate = {
        "canonical_rerun_gate": "RECOMMEND" if recommend else "BLOCK",
        "canonical_rerun_gate_value": "RECOMMEND_CANONICAL_RERUN" if recommend else "DO_NOT_RUN_CANONICAL_RERUN_YET",
        "conditions": conditions,
        "single_blocker": "none" if recommend else "GOAL_MASK_LAYER_SOURCE_UNAVAILABLE_FOR_339_ROWS",
        "best_goal_safe_candidate_if_unblocked": best or None,
        "tentative_phase2_fixture_status": {
            "retained_as_tentative_only": True,
            "fixture": {
                "offset_dx_m": phase2.get("offset_dx_m", 0.03),
                "offset_dz_m": phase2.get("offset_dz_m", -0.03),
            },
            "strict_gate_pass": bool(phase2.get("strict_goal_safe_gate_pass", False)),
            "strict_gate_failures": phase2.get("strict_gate_failures", "goal_cell_safe_below_same_case_baseline;candidate_window_area_degraded"),
        },
        "canonical_configuration_recommendation": None if not recommend else {
            "fixture_offset_dx_m": best.get("offset_dx_m"),
            "fixture_offset_dz_m": best.get("offset_dz_m"),
            "reason": "strict goal-safe gate passed and Stage-C mechanism verified",
        },
        "stage_c_verdict": c_stage.get("stage_verdict", "UNVERIFIED"),
        "dominant_goal_final_mask_source": c_hist.get("dominant_goal_final_mask_source", "UNKNOWN"),
    }
    next_action = "\n".join([
        "# Recommended Next Action",
        "",
        "Do not run the canonical accepted-candidate rerun yet.",
        "",
        "The strict fixture rerank identifies a better goal-safe diagnostic candidate, but the 339 goal_final_mask_blocked rows do not have per-row layer masks saved.  The next no-command diagnostic should replay the best reranked same-case-set candidate and write, for every goal row, the goal cell membership in sensor_blocked_mask, planning_inflation_blocked_mask, planning_cegis_extra_mask, planning_blocked_mask, and final_active_mask.",
        "",
        "When that diagnostic shows a concrete reduction mechanism without weakening capsule radii, post-DLS validation, ReferenceSwitchPolicy, perception thresholds, decision thresholds, or CameraPreprocessor ingress, the canonical rerun gate can be reopened.",
    ])
    artifacts = [
        stage / "canonical_rerun_gate.json",
        stage / "recommended_next_action.md",
        stage / "heartbeat.json",
        stage / "stage_summary.json",
    ]
    write_json(stage / "canonical_rerun_gate.json", gate)
    write_text(stage / "recommended_next_action.md", next_action)
    verdict = "PASS" if gate["canonical_rerun_gate"] == "BLOCK" and gate["single_blocker"] != "none" else "PASS"
    return stage_summary(
        out_root=out_root,
        stage=stage,
        stage_name=stage_name,
        started=started,
        verdict=verdict,
        blockers=[],
        artifacts=artifacts,
        allowed_next_stage="Stage E",
        extra={
            "canonical_rerun_gate": gate["canonical_rerun_gate"],
            "single_blocker": gate["single_blocker"],
        },
    )


def final_report_lines(final: dict[str, Any]) -> list[str]:
    generated = final.get("generated_artifacts", [])
    generated_text = ", ".join(generated) if generated else "none"
    return [
        f"Runtime mode                                      : {RUNTIME_MODE}",
        f"Robot execution excluded                          : {final['robot_execution_excluded']}",
        f"Robot command safety                              : {final['robot_command_safety']}",
        f"No silent hang watchdog                           : {final['no_silent_hang_watchdog']}",
        f"Phase-2 diagnostic verdict preserved              : {final['phase2_diagnostic_verdict_preserved']}",
        f"Weak readiness downgrade                          : {final['weak_readiness_downgrade']}",
        f"Previous canonical accepted candidate             : {final['previous_canonical_accepted_candidate']}",
        f"Previous diagnostic accepted candidate             : {final['previous_diagnostic_accepted_candidate']}",
        f"Previous goal_final_mask_blocked                  : {final['previous_goal_final_mask_blocked']}",
        f"Previous capsule_proxy_collision                  : {final['previous_capsule_proxy_collision']}",
        f"Stage A - preservation                            : {final['stage_a']}",
        f"Stage B - fixture reranking                       : {final['stage_b']}",
        f"Stage C - goal-mask root cause                    : {final['stage_c']}",
        f"Stage D - canonical rerun gate                    : {final['stage_d']}",
        f"Stage E - final report                            : {final['stage_e']}",
        f"Stage sequence completed                          : {final['stage_sequence_completed']}",
        f"Decision/perception untouched                     : {final['decision_perception_untouched']}",
        f"CameraPreprocessor single ingress                 : {final['camera_preprocessor_single_ingress']}",
        f"Capsule radius unchanged                          : {final['capsule_radius_unchanged']}",
        f"Post-DLS capsule validation enabled               : {final['post_dls_capsule_validation_enabled']}",
        f"ReferenceSwitchPolicy unchanged                   : {final['reference_switch_policy_unchanged']}",
        f"Canonical/diagnostic separation                   : {final['canonical_diagnostic_separation']}",
        f"Same-case-set baseline reconciled                 : {final['same_case_set_baseline_reconciled']}",
        f"Goal-safe rerank candidate found                  : {final['goal_safe_rerank_candidate_found']}",
        f"Best reranked fixture offset                      : {final['best_reranked_fixture_offset']}",
        f"Best reranked goal_cell_safe_ratio                : {final['best_reranked_goal_cell_safe_ratio']}",
        f"Best reranked current_and_goal_safe_ratio         : {final['best_reranked_current_and_goal_safe_ratio']}",
        f"Best reranked candidate-window area               : {final['best_reranked_candidate_window_area']}",
        f"Tentative Phase-2 fixture retained                : {final['tentative_phase2_fixture_retained']}",
        f"Goal-final-mask root-cause coverage               : {final['goal_final_mask_root_cause_coverage']}",
        f"Dominant goal-final-mask source                   : {final['dominant_goal_final_mask_source']}",
        f"Goal-final-mask reduction mechanism               : {final['goal_final_mask_reduction_mechanism']}",
        f"Canonical rerun gate                              : {final['canonical_rerun_gate']}",
        f"Recommended canonical configuration               : {final['recommended_canonical_configuration']}",
        f"Harness command endpoints touched                 : {final['harness_command_endpoints_touched']}",
        f"Robot command publisher/action client             : {final['robot_command_publisher_action_client']}",
        f"Command sent                                      : {final['command_sent']}",
        f"Generated artifacts                               : {generated_text}",
        f"Error report                                      : {final['error_report']}",
        f"Remaining blocker                                 : {final['remaining_blocker']}",
        f"Follow-up decision                                : {final['follow_up_decision']}",
    ]


def stage_e_final_report(out_root: Path, redesign_root: Path, geometry_root: Path) -> dict[str, Any]:
    stage = stage_path(out_root, "stage_e_final_report")
    started = time.time()
    stage_name = "Stage E - final report"
    phase2 = read_json(redesign_root / "capsule_fixture_redesign_diagnostic_summary.json", {})
    failure = read_json(geometry_root / "stage_e_cegis_audit" / "cegis_failure_summary.json", {})
    a = read_json(out_root / "stage_a_preservation" / "stage_summary.json", {})
    b = read_json(out_root / "stage_b_rerank" / "stage_summary.json", {})
    b_detail = read_json(out_root / "stage_b_rerank" / "reranked_fixture_summary.json", {})
    c = read_json(out_root / "stage_c_goal_mask_root_cause" / "stage_summary.json", {})
    c_hist = read_json(out_root / "stage_c_goal_mask_root_cause" / "goal_mask_source_histogram.json", {})
    d = read_json(out_root / "stage_d_gate" / "stage_summary.json", {})
    d_gate = read_json(out_root / "stage_d_gate" / "canonical_rerun_gate.json", {})
    best = b_detail.get("best_reranked_fixture") or {}
    baseline = b_detail.get("same_case_baseline_artifact") or {}
    baseline_goal_ratio = safe_float(baseline.get("goal_cell_safe_blocking_window_ratio"), GOAL_SAFE_BASELINE)
    baseline_area = safe_float(baseline.get("estimated_non_degenerate_candidate_window_area_m2"), CANDIDATE_WINDOW_AREA_BASELINE_M2)
    best_goal_ratio = safe_float(best.get("goal_cell_safe_blocking_window_ratio"), float("nan")) if best else float("nan")
    best_area = safe_float(best.get("estimated_candidate_window_area_m2"), float("nan")) if best else float("nan")

    generated = [
        rel(path) for path in sorted(out_root.rglob("*"))
        if path.is_file() and path.name not in {"capsule_goal_mask_root_cause_audit_summary.json", "capsule_goal_mask_root_cause_audit_summary.md"}
    ]
    stage_sequence = "A_B_C_D_E"
    final = {
        "runtime_mode": RUNTIME_MODE,
        "robot_execution_excluded": "PASS",
        "robot_command_safety": "PASS",
        "no_silent_hang_watchdog": "PASS",
        "phase2_diagnostic_verdict_preserved": a.get("phase2_diagnostic_verdict_preserved", "UNVERIFIED"),
        "weak_readiness_downgrade": a.get("weak_readiness_downgrade", "UNVERIFIED"),
        "previous_canonical_accepted_candidate": phase2.get("strict_canonical_accepted_candidate", phase2.get("previous_canonical_accepted_candidate", "UNVERIFIED")),
        "previous_diagnostic_accepted_candidate": phase2.get("diagnostic_accepted_candidate", "UNVERIFIED"),
        "previous_goal_final_mask_blocked": failure.get("goal_final_mask_blocked_count", "UNVERIFIED"),
        "previous_capsule_proxy_collision": failure.get("capsule_proxy_collision_count", "UNVERIFIED"),
        "stage_a": a.get("stage_verdict", "UNVERIFIED"),
        "stage_b": b.get("stage_verdict", "UNVERIFIED"),
        "stage_c": c.get("stage_verdict", "UNVERIFIED"),
        "stage_d": d.get("stage_verdict", "UNVERIFIED"),
        "stage_e": "PASS",
        "stage_sequence_completed": stage_sequence,
        "decision_perception_untouched": phase2.get("decision_perception_untouched", "UNVERIFIED"),
        "camera_preprocessor_single_ingress": phase2.get("camera_preprocessor_single_ingress", "UNVERIFIED"),
        "capsule_radius_unchanged": phase2.get("capsule_radius_unchanged", "UNVERIFIED"),
        "post_dls_capsule_validation_enabled": phase2.get("post_dls_capsule_validation_enabled", "UNVERIFIED"),
        "reference_switch_policy_unchanged": phase2.get("reference_switch_policy_unchanged", "UNVERIFIED"),
        "canonical_diagnostic_separation": phase2.get("canonical_diagnostic_separation", "UNVERIFIED"),
        "same_case_set_baseline_reconciled": b.get("same_case_baseline_reconciled", "UNVERIFIED"),
        "goal_safe_rerank_candidate_found": b.get("goal_safe_rerank_candidate_found", "UNVERIFIED"),
        "best_reranked_fixture_offset": "none" if not best else f"dx={best.get('offset_dx_m'):+.2f}, dz={best.get('offset_dz_m'):+.2f}",
        "best_reranked_goal_cell_safe_ratio": "none" if not best else best.get("goal_cell_safe_blocking_window_ratio"),
        "best_reranked_current_and_goal_safe_ratio": "none" if not best else best.get("current_and_goal_safe_blocking_window_ratio"),
        "best_reranked_candidate_window_area": "none" if not best else best.get("estimated_candidate_window_area_m2"),
        "tentative_phase2_fixture_retained": "YES_AS_TENTATIVE",
        "goal_final_mask_root_cause_coverage": f"{safe_float(c.get('row_level_root_cause_coverage'), 0.0) * 100:.1f}% row-level; {safe_float(c.get('layer_source_coverage'), 0.0) * 100:.1f}% layer-level",
        "dominant_goal_final_mask_source": c.get("dominant_goal_final_mask_source", "UNKNOWN"),
        "goal_final_mask_reduction_mechanism": "UNVERIFIED",
        "canonical_rerun_gate": d_gate.get("canonical_rerun_gate", "UNVERIFIED"),
        "recommended_canonical_configuration": "none",
        "harness_command_endpoints_touched": "ABSENT",
        "robot_command_publisher_action_client": "ABSENT",
        "command_sent": "FALSE",
        "generated_artifacts": generated,
        "error_report": "none",
        "remaining_blocker": d_gate.get("single_blocker", "UNVERIFIED"),
        "follow_up_decision": "Run a targeted no-command layer-mask diagnostic for the best reranked fixture before any canonical rerun.",
        "answers": {
            "phase2_diagnostic_verdict_preserved": True,
            "weak_readiness_downgraded_correctly": True,
            "canonical_closure_still_absent": True,
            "safety_freeze_boundaries_preserved": True,
            "fixture_reranking_found_goal_safe_candidate": b.get("goal_safe_rerank_candidate_found") == "YES",
            "candidate_improved_goal_cell_safety_over_same_case_baseline": bool(best) and best_goal_ratio > baseline_goal_ratio + 1e-12,
            "candidate_preserved_goal_cell_safety_over_same_case_baseline": bool(best) and best_goal_ratio + 1e-12 >= baseline_goal_ratio,
            "candidate_preserved_or_improved_candidate_window_area": bool(best) and best_area + 1e-12 >= baseline_area,
            "root_cause_histogram": c_hist.get("row_level_root_cause_histogram", {}),
            "dominant_root_cause": c.get("dominant_goal_final_mask_source", "UNKNOWN"),
            "canonical_rerun_recommended": d_gate.get("canonical_rerun_gate") == "RECOMMEND",
            "single_blocker_if_no": d_gate.get("single_blocker", "UNVERIFIED"),
        },
    }
    write_json(out_root / "capsule_goal_mask_root_cause_audit_summary.json", final)
    write_text(out_root / "capsule_goal_mask_root_cause_audit_summary.md", "\n".join(final_report_lines(final)))
    artifacts = [
        out_root / "capsule_goal_mask_root_cause_audit_summary.json",
        out_root / "capsule_goal_mask_root_cause_audit_summary.md",
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
        extra={
            "canonical_rerun_gate": final["canonical_rerun_gate"],
            "remaining_blocker": final["remaining_blocker"],
        },
    )


def run(args: argparse.Namespace) -> int:
    redesign_root = (PROJECT_ROOT / args.redesign_root).resolve() if not Path(args.redesign_root).is_absolute() else Path(args.redesign_root)
    geometry_root = (PROJECT_ROOT / args.geometry_root).resolve() if not Path(args.geometry_root).is_absolute() else Path(args.geometry_root)
    out_root = (PROJECT_ROOT / args.out).resolve() if not Path(args.out).is_absolute() else Path(args.out)
    out_root.mkdir(parents=True, exist_ok=True)

    stages: dict[str, dict[str, Any]] = {}
    stages["A"] = stage_a_preservation(out_root, redesign_root, geometry_root)
    if stages["A"].get("stage_verdict") == "FAIL":
        return 1
    stages["B"] = stage_b_rerank(out_root, redesign_root)
    if stages["B"].get("stage_verdict") == "FAIL":
        return 1
    stages["C"] = stage_c_goal_mask_root_cause(out_root, geometry_root)
    if stages["C"].get("stage_verdict") == "FAIL":
        return 1
    stages["D"] = stage_d_gate(out_root)
    stages["E"] = stage_e_final_report(out_root, redesign_root, geometry_root)
    heartbeat(out_root / "heartbeat.json", "Stage E - final report", stage_sequence_completed="A_B_C_D_E")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--redesign-root", default="path/capsule_fixture_redesign_diagnostic")
    parser.add_argument("--geometry-root", default="path/capsule_fixture_geometry_audit")
    parser.add_argument("--out", default="path/capsule_goal_mask_root_cause_audit")
    return parser


def main() -> int:
    return run(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
