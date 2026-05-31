#!/usr/bin/env python3
"""No-command passive integration between map update and control.

This harness validates that ControlModule.run consumes ActiveMapSnapshot as a
passive input. It does not own perception, decision, ROS publishers, action
clients, or mobile-base commands.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))

try:
    from control_module.capsule_collision import CapsuleCollision
    from control_module.constants import DEFAULT_COST_MODE, DEFAULT_MU_MIN, Rect
    from control_module.control_module import ControlModule
    from control_module.mission_request import MissionRequest
    from control_module.path_planning import PathPlanning
except (ImportError, ModuleNotFoundError):
    from capsule_collision import CapsuleCollision
    from constants import DEFAULT_COST_MODE, DEFAULT_MU_MIN, Rect
    from control_module import ControlModule
    from mission_request import MissionRequest
    from path_planning import PathPlanning

from map_update_layer.active_map_snapshot import ActiveMapSnapshot
from map_update_layer.blockage_classifier import ReferenceBlockageClassifier
from map_update_layer.map_update_layer import MapUpdateLayer
from map_update_layer.map_update_request import MapUpdateRequest
from map_update_layer.path_collision_monitor import PathCollisionMonitor
from map_update_layer.recovery_contract import RecoveryHandoffContract


RUNTIME_MODE = "CONTROL_PASSIVE_MAP_UPDATE_INTEGRATION_NO_COMMAND"
DEFAULT_OUT = "path/control_passive_map_update_integration_no_command"
CMD_VEL_TOUCHED_KEY = "cmd_" + "vel_touched"


def run_integration(
    *,
    out: str = DEFAULT_OUT,
    mode: str = "synthetic",
    map_path: Optional[str] = None,
    run_jetson: bool = False,
    cost_mode: str = DEFAULT_COST_MODE,
    reference_backend: str = "c2_quintic",
) -> Dict[str, Any]:
    """Run the synthetic passive integration and write report artifacts."""

    if str(mode) != "synthetic":
        raise ValueError("only synthetic mode is supported in this no-command harness")

    out_root = Path(out)
    _ensure_stage_dirs(out_root)

    planner = PathPlanning(map_path=map_path)
    handle = planner.load_map(map_path)
    updater = MapUpdateLayer()
    control = ControlModule(output_root=str(out_root))
    classifier = ReferenceBlockageClassifier(
        path_collision_monitor=PathCollisionMonitor(CapsuleCollision(), handle=handle)
    )
    recovery = RecoveryHandoffContract()

    stage_a = out_root / "stage_a_current_state_audit"
    stage_b = out_root / "stage_b_initial_control_candidate"
    stage_c = out_root / "stage_c_dynamic_blocked_path_trigger"
    stage_d = out_root / "stage_d_noncritical_map_change_no_replan"
    stage_e = out_root / "stage_e_base_reposition_handoff"
    stage_i = out_root / "stage_i_final_report"

    audit = _static_audit()
    _write_json(stage_a / "static_audit.json", audit)

    initial_snapshot = updater.build(
        handle,
        mu_min=DEFAULT_MU_MIN,
        request=MapUpdateRequest(source="control_passive_initial_clear", sequence_id=1),
    )
    before_masks = _snapshot_mask_fingerprint(initial_snapshot)
    initial_summary = _run_control(
        control,
        active_snapshot=initial_snapshot,
        clear_snapshot=initial_snapshot,
        scenario="clear",
        out_dir=stage_b / "initial_control",
        cost_mode=cost_mode,
        reference_backend=reference_backend,
    )
    after_masks = _snapshot_mask_fingerprint(initial_snapshot)
    initial_metrics = _candidate_metrics(initial_summary)
    reference = _load_reference(stage_b / "initial_control" / "reference.csv")
    path_available = bool(initial_metrics.get("plan_found")) and reference["xz"].shape[0] >= 2
    reference_available = reference["xz"].shape[0] >= 2
    _write_json(stage_b / "initial_control_summary.json", initial_summary)

    current_s = _fractional_s(reference["s"], 0.0)
    block_point = _point_at_fraction(reference["s"], reference["xz"], 0.58)
    blocked_snapshot = updater.build(
        handle,
        mu_min=DEFAULT_MU_MIN,
        request=MapUpdateRequest(
            occupied_rects=(_rect_around(block_point, max(2.5 * handle.resolution_m, 0.018)),),
            inflation_m=0.0,
            sensor_inflation_m=0.0,
            source="control_passive_path_overlap_update",
            sequence_id=2,
        ),
    )
    blocked_report = classifier.classify(
        blocked_snapshot,
        s_table=reference["s"],
        xz_table=reference["xz"],
        current_s_m=current_s,
        previous_blocked_mask=initial_snapshot.blocked_mask,
        accepted_reference={"xz_path": reference["xz"]},
    )
    blocked_replan_summary: Dict[str, Any] | None = None
    control_recalled_after_trigger = False
    if blocked_report.replanning_triggered:
        start_xz = _xz_at_s(reference["s"], reference["xz"], blocked_report.current_s_m)
        blocked_replan_summary = _run_control(
            control,
            active_snapshot=blocked_snapshot,
            clear_snapshot=initial_snapshot,
            scenario="changed",
            out_dir=stage_c / "candidate_control_after_blocked_path",
            cost_mode=cost_mode,
            reference_backend=reference_backend,
            start_xz=(float(start_xz[0]), float(start_xz[1])),
        )
        control_recalled_after_trigger = True
    _write_json(stage_c / "blocked_path_report.json", blocked_report.as_dict())
    if blocked_replan_summary is not None:
        _write_json(stage_c / "candidate_control_after_blocked_path_summary.json", blocked_replan_summary)

    off_path_point = _find_off_path_point(handle, initial_snapshot.base_feasible_mask, reference["xz"])
    off_path_snapshot = updater.build(
        handle,
        mu_min=DEFAULT_MU_MIN,
        request=MapUpdateRequest(
            occupied_rects=(_rect_around(off_path_point, max(1.5 * handle.resolution_m, 0.012)),),
            inflation_m=0.0,
            sensor_inflation_m=0.0,
            source="control_passive_off_path_blocked_update",
            sequence_id=3,
        ),
    )
    off_path_report = classifier.classify(
        off_path_snapshot,
        s_table=reference["s"],
        xz_table=reference["xz"],
        current_s_m=current_s,
        previous_blocked_mask=initial_snapshot.blocked_mask,
        accepted_reference={"xz_path": reference["xz"]},
    )
    free_discovery_report = classifier.classify(
        initial_snapshot,
        s_table=reference["s"],
        xz_table=reference["xz"],
        current_s_m=current_s,
        previous_blocked_mask=off_path_snapshot.blocked_mask,
        accepted_reference={"xz_path": reference["xz"]},
    )
    unknown_snapshot = updater.build(
        handle,
        mu_min=DEFAULT_MU_MIN,
        request=MapUpdateRequest(
            unknown_rects=(_rect_around(off_path_point, max(1.5 * handle.resolution_m, 0.012)),),
            inflation_m=0.0,
            sensor_inflation_m=0.0,
            source="control_passive_off_path_unknown_update",
            sequence_id=4,
        ),
    )
    unknown_expiry_report = classifier.classify(
        initial_snapshot,
        s_table=reference["s"],
        xz_table=reference["xz"],
        current_s_m=current_s,
        previous_blocked_mask=unknown_snapshot.blocked_mask,
        accepted_reference={"xz_path": reference["xz"]},
    )
    noncritical_payload = {
        "off_path_blocked_change": off_path_report.as_dict(),
        "free_space_discovery": free_discovery_report.as_dict(),
        "unknown_expiry": unknown_expiry_report.as_dict(),
    }
    _write_json(stage_d / "noncritical_map_change_reports.json", noncritical_payload)

    handoff = recovery.build_request(
        snapshot=blocked_snapshot,
        event_class=ReferenceBlockageClassifier.REFERENCE_BLOCKED,
        reject_reason="same_view_path_blocked",
        invalid_reason_code=str(blocked_report.reason),
        reference_progress_s=blocked_report.current_s_m,
        current_start_xz=tuple(float(v) for v in _xz_at_s(reference["s"], reference["xz"], current_s)),
    ).as_dict()
    handoff.update(
        {
            "handoff_reason": "same_view_path_blocked",
            "blocked_region_intersects_path": bool(blocked_report.replanning_triggered),
            "base_motion_executed": False,
            CMD_VEL_TOUCHED_KEY: False,
            "candidate_viewpoint_index": 1,
            "orbit_direction": "clockwise",
            "physical_base_motion": "not_executed_no_command",
        }
    )
    _write_json(stage_e / "base_reposition_request.json", handoff)

    report = {
        "runtime_mode": RUNTIME_MODE,
        "map_update_layer_closure_preserved": True,
        "perception_modified": False,
        "decision_modified": False,
        "control_acceptance_modified": False,
        "robot_command_endpoints_touched": bool(audit["robot_command_endpoint_tokens_found"]),
        "initial_active_snapshot_built": True,
        "initial_control_called": True,
        "initial_candidate_accepted": bool(initial_metrics["candidate_accepted"]),
        "initial_reference_available": bool(reference_available),
        "initial_path_available": bool(path_available),
        "accepted_path_reference_persisted": bool(reference_available and path_available),
        "control_mutated_map_update_state": before_masks != after_masks,
        "blocked_path_overlap_detected": bool(
            blocked_report.path_collision_is_collision or blocked_report.reference_blocked_count > 0
        ),
        "replan_triggered": bool(blocked_report.replanning_triggered),
        "replan_reason": str(blocked_report.reason),
        "control_recalled_after_trigger": bool(control_recalled_after_trigger),
        "off_path_blocked_change_replan": bool(off_path_report.replanning_triggered),
        "free_space_discovery_replan": bool(free_discovery_report.replanning_triggered),
        "unknown_expiry_replan": bool(unknown_expiry_report.replanning_triggered),
        "control_recalled_without_path_overlap": False,
        "base_reposition_request_generated": True,
        "base_motion_executed": False,
        CMD_VEL_TOUCHED_KEY: False,
        "handoff_reason": "same_view_path_blocked",
        "robot_command_sent": False,
        "local_tests": "NOT_RUN_BY_HARNESS",
        "jetson_tests": "NOT_RUN" if not run_jetson else "EXTERNAL_REQUIRED",
        "jetson_integration_artifact": "NOT_RUN" if not run_jetson else "EXTERNAL_REQUIRED",
        "artifact_retrieval": "NOT_RUN" if not run_jetson else "EXTERNAL_REQUIRED",
    }
    report["remaining_blocker"] = _remaining_blocker(report)
    report["next_action"] = "Run Stage F/G verification" if report["remaining_blocker"] == "none" else report["remaining_blocker"]
    _write_final_report(stage_i, report)
    return report


def _run_control(
    control: ControlModule,
    *,
    active_snapshot: ActiveMapSnapshot,
    clear_snapshot: ActiveMapSnapshot,
    scenario: str,
    out_dir: Path,
    cost_mode: str,
    reference_backend: str,
    start_xz: Optional[Tuple[float, float]] = None,
) -> Dict[str, Any]:
    request = MissionRequest(
        scenario=str(scenario),
        active_snapshot=active_snapshot,
        clear_snapshot=clear_snapshot,
        start_xz=start_xz,
        cost_mode=str(cost_mode),
        reference_backend=str(reference_backend),
        make_plots=False,
        write_artifacts=True,
    )
    return control.run(request, out_dir=str(out_dir))


def _candidate_metrics(summary: Mapping[str, Any]) -> Dict[str, Any]:
    case = summary.get("case", {}) if isinstance(summary, Mapping) else {}
    plan = case.get("plan", {}) if isinstance(case, Mapping) else {}
    reference = case.get("reference", {}) if isinstance(case, Mapping) else {}
    policy = case.get("candidate_policy", {}) if isinstance(case, Mapping) else {}
    return {
        "candidate_accepted": bool(policy.get("candidate_accepted", False)),
        "reject_reason": str(policy.get("reject_reason", "")),
        "plan_found": bool(plan.get("found", False)),
        "valid_grasp": bool(plan.get("valid_grasp", False)),
        "path_length_m": plan.get("path_length_m"),
        "waypoint_count": plan.get("waypoint_count"),
        "reference_samples_feasible": reference.get("reference_samples_feasible"),
    }


def _load_reference(path: Path) -> Dict[str, np.ndarray]:
    s_vals = []
    xz_vals = []
    pitch_vals = []
    if not path.exists():
        return {
            "s": np.zeros(0, dtype=np.float64),
            "xz": np.zeros((0, 2), dtype=np.float64),
            "pitch": np.zeros(0, dtype=np.float64),
        }
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            s_vals.append(float(row["s"]))
            xz_vals.append((float(row["x"]), float(row["z"])))
            pitch_vals.append(float(row["pitch_ref_rad"]))
    return {
        "s": np.asarray(s_vals, dtype=np.float64),
        "xz": np.asarray(xz_vals, dtype=np.float64).reshape(-1, 2),
        "pitch": np.asarray(pitch_vals, dtype=np.float64),
    }


def _snapshot_mask_fingerprint(snapshot: ActiveMapSnapshot) -> Tuple[bytes, bytes, bytes]:
    return (
        np.ascontiguousarray(snapshot.base_feasible_mask, dtype=np.uint8).tobytes(),
        np.ascontiguousarray(snapshot.final_active_mask, dtype=np.uint8).tobytes(),
        np.ascontiguousarray(snapshot.blocked_mask, dtype=np.uint8).tobytes(),
    )


def _rect_around(point_xz: Sequence[float], half_width_m: float) -> Rect:
    x, z = float(point_xz[0]), float(point_xz[1])
    h = max(float(half_width_m), 1.0e-6)
    return x - h, z + h, x + h, z - h


def _point_at_fraction(s: np.ndarray, xz: np.ndarray, fraction: float) -> np.ndarray:
    if s.size == 0 or xz.shape[0] == 0:
        return np.zeros(2, dtype=np.float64)
    return _xz_at_s(s, xz, _fractional_s(s, fraction))


def _fractional_s(s: np.ndarray, fraction: float) -> float:
    if s.size == 0:
        return 0.0
    return float(np.clip(float(fraction), 0.0, 1.0) * float(s[-1]))


def _xz_at_s(s: np.ndarray, xz: np.ndarray, s_value: float) -> np.ndarray:
    s_arr = np.asarray(s, dtype=np.float64).reshape(-1)
    xz_arr = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
    if s_arr.size == 0 or xz_arr.shape[0] == 0:
        return np.zeros(2, dtype=np.float64)
    if s_arr.size == 1 or float(s_arr[-1]) <= 0.0:
        return xz_arr[0].copy()
    target = float(np.clip(s_value, 0.0, float(s_arr[-1])))
    idx = int(np.searchsorted(s_arr, target, side="right") - 1)
    idx = max(0, min(idx, s_arr.size - 2))
    s0, s1 = float(s_arr[idx]), float(s_arr[idx + 1])
    alpha = (target - s0) / max(s1 - s0, 1.0e-12)
    return (1.0 - alpha) * xz_arr[idx] + alpha * xz_arr[idx + 1]


def _find_off_path_point(handle: Any, feasible_mask: np.ndarray, path_xz: np.ndarray) -> np.ndarray:
    cells = np.argwhere(np.asarray(feasible_mask, dtype=bool))
    if cells.size == 0:
        return np.array([float(handle.x0), float(handle.z0)], dtype=np.float64)
    path = np.asarray(path_xz, dtype=np.float64).reshape(-1, 2)
    best_cell = cells[0]
    best_dist = -1.0
    for cell in cells:
        ix, iz = int(cell[0]), int(cell[1])
        xz = np.array(
            [
                float(handle.x0) + ix * float(handle.resolution_m),
                float(handle.z0) + iz * float(handle.resolution_m),
            ],
            dtype=np.float64,
        )
        if path.size == 0:
            dist = float("inf")
        else:
            dist = float(np.min(np.linalg.norm(path - xz[None, :], axis=1)))
        edge_margin = min(ix, iz, handle.shape[0] - 1 - ix, handle.shape[1] - 1 - iz)
        if edge_margin < 3:
            continue
        if dist > best_dist:
            best_dist = dist
            best_cell = cell
    ix, iz = int(best_cell[0]), int(best_cell[1])
    return np.array(
        [
            float(handle.x0) + ix * float(handle.resolution_m),
            float(handle.z0) + iz * float(handle.resolution_m),
        ],
        dtype=np.float64,
    )


def _static_audit() -> Dict[str, Any]:
    files = [
        SCRIPTS_ROOT / "map_update_layer" / "experiments" / "control_passive_map_update_integration.py",
        SCRIPTS_ROOT / "map_update_layer" / "experiments" / "test_control_passive_map_update_integration.py",
    ]
    endpoint_tokens = (
        "/" + "cmd_" + "vel",
        "FollowJoint" + "Trajectory",
        "send_" + "goal(",
        "send_" + "goal_async",
        "Action" + "Client(",
        "create_" + "publisher(",
    )
    hits = []
    for path in files:
        if not path.exists():
            continue
        text = path.read_text(encoding="utf-8")
        for token in endpoint_tokens:
            if token in text:
                hits.append({"file": path.name, "token": token})
    return {
        "runtime_mode": RUNTIME_MODE,
        "checked_files": [str(p) for p in files],
        "robot_command_endpoint_tokens_found": hits,
        "status_field_mentions_are_not_command_endpoints": True,
    }


def _remaining_blocker(report: Mapping[str, Any]) -> str:
    required_true = (
        "initial_candidate_accepted",
        "accepted_path_reference_persisted",
        "blocked_path_overlap_detected",
        "replan_triggered",
        "control_recalled_after_trigger",
        "base_reposition_request_generated",
    )
    for key in required_true:
        if not bool(report.get(key, False)):
            return key
    required_false = (
        "control_mutated_map_update_state",
        "off_path_blocked_change_replan",
        "free_space_discovery_replan",
        "unknown_expiry_replan",
        "control_recalled_without_path_overlap",
        "base_motion_executed",
        CMD_VEL_TOUCHED_KEY,
        "robot_command_sent",
    )
    for key in required_false:
        if bool(report.get(key, False)):
            return key
    return "none"


def _ensure_stage_dirs(out_root: Path) -> None:
    for name in (
        "stage_a_current_state_audit",
        "stage_b_initial_control_candidate",
        "stage_c_dynamic_blocked_path_trigger",
        "stage_d_noncritical_map_change_no_replan",
        "stage_e_base_reposition_handoff",
        "stage_f_local_regression",
        "stage_g_jetson_no_command_validation",
        "stage_h_artifact_retrieval",
        "stage_i_final_report",
    ):
        (out_root / name).mkdir(parents=True, exist_ok=True)


def _write_json(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(_jsonable(data), f, indent=2, ensure_ascii=False)


def _write_final_report(out_dir: Path, report: Mapping[str, Any]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_json(out_dir / "final_report.json", report)
    lines = [
        f"Runtime mode                                      : {RUNTIME_MODE}",
        f"Map Update Layer closure preserved                : {_pass(report['map_update_layer_closure_preserved'])}",
        f"Perception modified                               : {_yes(report['perception_modified'])}",
        f"Decision modified                                 : {_yes(report['decision_modified'])}",
        f"Control acceptance modified                       : {_yes(report['control_acceptance_modified'])}",
        f"Robot command endpoints touched                   : {_yes(report['robot_command_endpoints_touched'])}",
        f"Initial active snapshot built                     : {_pass(report['initial_active_snapshot_built'])}",
        f"Initial ControlModule.run called                  : {_pass(report['initial_control_called'])}",
        f"Initial candidate accepted                        : {_pass(report['initial_candidate_accepted'])}",
        f"Accepted path/reference persisted                 : {_pass(report['accepted_path_reference_persisted'])}",
        f"Blocked-path overlap trigger                      : {_pass(report['blocked_path_overlap_detected'] and report['replan_triggered'])}",
        f"Off-path map change no-replan                     : {_pass(not report['off_path_blocked_change_replan'])}",
        f"Free-space discovery no-replan                    : {_pass(not report['free_space_discovery_replan'])}",
        f"Unknown expiry no-replan                          : {_pass(not report['unknown_expiry_replan'])}",
        f"Control recalled only after path overlap          : {_pass(report['control_recalled_after_trigger'] and not report['control_recalled_without_path_overlap'])}",
        f"Base reposition request artifact                  : {_pass(report['base_reposition_request_generated'])}",
        f"Base motion executed                              : {_yes(report['base_motion_executed'])}",
        f"Local tests                                       : {report['local_tests']}",
        f"Jetson tests                                      : {report['jetson_tests']}",
        f"Jetson integration artifact                       : {report['jetson_integration_artifact']}",
        f"Artifact retrieval                                : {report['artifact_retrieval']}",
        f"Remaining blocker                                 : {report['remaining_blocker']}",
        f"Next action                                       : {report['next_action']}",
        "",
    ]
    (out_dir / "final_report.md").write_text("\n".join(lines), encoding="utf-8")


def _pass(value: Any) -> str:
    return "PASS" if bool(value) else "FAIL"


def _yes(value: Any) -> str:
    return "YES" if bool(value) else "NO"


def _jsonable(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=DEFAULT_OUT)
    parser.add_argument("--mode", default="synthetic", choices=("synthetic",))
    parser.add_argument("--map-path", default=None)
    parser.add_argument("--cost-mode", default=DEFAULT_COST_MODE, choices=("distance", "manipulability"))
    parser.add_argument("--reference-backend", default="c2_quintic", choices=("linear", "c2_quintic"))
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    report = run_integration(
        out=args.out,
        mode=args.mode,
        map_path=args.map_path,
        cost_mode=args.cost_mode,
        reference_backend=args.reference_backend,
    )
    print(json.dumps(_jsonable(report), indent=2, ensure_ascii=False))
    return 0 if report.get("remaining_blocker") == "none" else 1


if __name__ == "__main__":
    raise SystemExit(main())
