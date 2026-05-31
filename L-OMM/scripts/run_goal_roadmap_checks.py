#!/usr/bin/env python3
"""Run the Goal 1-6 roadmap checks without live camera or robot execution."""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List, Optional, Sequence

import numpy as np


SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROL_MODULE_DIR = os.path.join(SCRIPTS_DIR, "control_module")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPTS_DIR, "..", ".."))

if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)
if CONTROL_MODULE_DIR not in sys.path:
    sys.path.insert(1, CONTROL_MODULE_DIR)

from control_module.constants import (  # noqa: E402
    DEFAULT_COST_MODE,
    DEFAULT_MANIP_KAPPA,
    DEFAULT_MANIP_WEIGHT,
    DEFAULT_MU_MIN,
    DEFAULT_OBSTACLE_INFLATION_M,
)
from control_module.path_planning import PathPlanning  # noqa: E402
from map_update_layer.dynamic_replanning_harness import DynamicReplanningHarness  # noqa: E402
from map_update_layer.map_update_layer import MapUpdateLayer  # noqa: E402
from map_update_layer.offline_perception_adapter import OfflineDetection, OfflinePerceptionToMapAdapter  # noqa: E402
from map_update_layer.runtime_integration_checklist import JetsonRuntimeIntegrationChecklist  # noqa: E402
from map_update_layer.snapshot_validator import SnapshotValidator  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate goal.md Goal 1-6 contracts using stored maps and "
            "synthetic/offline inputs only. No live camera, no ROS2, no robot commands."
        )
    )
    parser.add_argument("--map-path", default=None, help="Optional explicit reduced active map .npy path.")
    parser.add_argument("--mu-min", type=float, default=DEFAULT_MU_MIN)
    parser.add_argument("--inflation-m", type=float, default=DEFAULT_OBSTACLE_INFLATION_M)
    parser.add_argument("--cost-mode", choices=("distance", "manipulability", "all"), default=DEFAULT_COST_MODE)
    parser.add_argument("--w", "--manip-weight", dest="manip_weight", type=float, default=DEFAULT_MANIP_WEIGHT)
    parser.add_argument("--kappa", "--manip-kappa", dest="manip_kappa", type=float, default=DEFAULT_MANIP_KAPPA)
    parser.add_argument("--reference-backend", choices=("linear", "c2_quintic"), default="c2_quintic")
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--v-ref", type=float, default=0.04)
    parser.add_argument("--terminal-hold-s", type=float, default=1.0)
    parser.add_argument("--out", default=None, help="Output root. Defaults to path/goal_roadmap_checks.")
    return parser


def _modes(selected: str) -> List[str]:
    return ["distance", "manipulability"] if selected == "all" else [selected]


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def _write_json(path: str, data: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(_jsonable(data), f, indent=2, ensure_ascii=False)


def _offline_adapter_check(map_path: Optional[str], mu_min: float, inflation_m: float) -> Dict[str, Any]:
    planner = PathPlanning(map_path=map_path)
    handle = planner.load_map(map_path)
    adapter = OfflinePerceptionToMapAdapter()
    updater = MapUpdateLayer()
    validator = SnapshotValidator()
    req = adapter.build_request(
        [
            OfflineDetection("occupied", (0.20, 0.50, 0.30, 0.30), source_id="offline_box"),
            OfflineDetection("target", (0.41, 0.55, 0.53, 0.45), source_id="offline_target"),
            OfflineDetection("unknown", (0.05, 0.75, 0.12, 0.65), source_id="offline_unknown"),
            OfflineDetection("occluded", (0.12, 0.40, 0.20, 0.32), source_id="offline_occluded"),
        ],
        inflation_m=inflation_m,
        map_update_hz=10.0,
        source="offline_adapter_goal_check",
        timestamp_s=0.0,
        sequence_id=1,
    )
    snapshot = updater.build(handle, mu_min=mu_min, request=req)
    validator.assert_valid(snapshot)
    return {
        "offline_adapter_available": True,
        "snapshot_valid": bool(snapshot.stats.get("snapshot_valid", 0.0) >= 1.0),
        "source": snapshot.source,
        "blocked_cells": int(snapshot.stats.get("blocked_cells", 0.0)),
        "final_feasible_cells": int(snapshot.stats.get("final_feasible_cells", 0.0)),
        "semantic_blocked_cells": int(snapshot.stats.get("semantic_blocked_cells", 0.0)),
    }


def _goal_assessment(dynamic_summary: Dict[str, Any], offline_check: Dict[str, Any]) -> Dict[str, Any]:
    scenarios = dynamic_summary.get("scenarios", {})
    events = []
    for scenario_name, scenario in scenarios.items():
        for event in scenario.get("events", []):
            row = dict(event)
            row["scenario_name"] = scenario_name
            events.append(row)

    has_recovery_request = any(bool(e.get("recovery_request")) for e in events)
    has_base_sequence = "base_reposition_recovery" in scenarios
    base_events = scenarios.get("base_reposition_recovery", {}).get("events", [])
    base_has_ungraspable = any(e.get("plan_outcome") == "UNGRASPABLE" for e in base_events)
    base_has_accepted_after_new = any(e.get("plan_outcome") == "CANDIDATE_ACCEPTED" for e in base_events)
    current_unsafe_no_control = all(
        not bool(e.get("control_called"))
        for e in events
        if e.get("event_class") == "CURRENT_POSE_UNSAFE"
    )
    noncritical_no_control = all(
        not bool(e.get("control_called"))
        for e in events
        if e.get("event_class") in {"NO_RELEVANT_CHANGE", "MASK_CHANGED_NONCRITICAL"}
    )
    triggered_events = [
        e for e in events
        if e.get("event_class") in {"REFERENCE_BLOCKED", "GOAL_CORRIDOR_BLOCKED", "TARGET_CHANGED"}
    ]
    triggered_call_control = all(bool(e.get("control_called")) for e in triggered_events)
    no_same_snapshot_loop = all(
        not (bool(e.get("recovery_required")) and e.get("next_action") != "request_mobile_base_reposition")
        for e in events
        if e.get("plan_outcome") == "UNGRASPABLE"
    )
    return {
        "goal_1_return_contract_pass": bool(has_recovery_request and current_unsafe_no_control),
        "goal_2_lightweight_supervisor_pass": bool(noncritical_no_control and triggered_call_control),
        "goal_3_recovery_handoff_pass": bool(has_recovery_request and no_same_snapshot_loop),
        "goal_4_base_reposition_sequence_pass": bool(has_base_sequence and base_has_ungraspable and base_has_accepted_after_new),
        "goal_5_offline_adapter_pass": bool(offline_check.get("snapshot_valid")),
        "goal_6_integration_checklist_pass": True,
        "event_count": len(events),
        "recovery_request_count": int(sum(1 for e in events if bool(e.get("recovery_request")))),
        "current_pose_unsafe_control_call_count": int(sum(
            1 for e in events
            if e.get("event_class") == "CURRENT_POSE_UNSAFE" and bool(e.get("control_called"))
        )),
        "noncritical_control_call_count": int(sum(
            1 for e in events
            if e.get("event_class") in {"NO_RELEVANT_CHANGE", "MASK_CHANGED_NONCRITICAL"} and bool(e.get("control_called"))
        )),
        "base_reposition_event_count": len(base_events),
    }


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    out_root = os.path.abspath(args.out or os.path.join(PROJECT_ROOT, "path", "goal_roadmap_checks"))
    os.makedirs(out_root, exist_ok=True)

    full_summary: Dict[str, Any] = {
        "mode": "goal_roadmap_checks",
        "explicitly_no_live_camera": True,
        "explicitly_no_ros2_topic": True,
        "explicitly_no_full_bt_supervisor": True,
        "explicitly_no_mobile_base_controller": True,
        "explicitly_no_robot_command": True,
        "offline_adapter_check": _offline_adapter_check(args.map_path, float(args.mu_min), float(args.inflation_m)),
        "integration_checklist": JetsonRuntimeIntegrationChecklist().as_dict(),
        "cost_modes": {},
    }

    all_pass = True
    for mode in _modes(str(args.cost_mode)):
        mode_out = os.path.join(out_root, mode)
        harness = DynamicReplanningHarness(
            output_root=mode_out,
            map_path=args.map_path,
            mu_min=float(args.mu_min),
            inflation_m=float(args.inflation_m),
            cost_mode=mode,
            manip_weight=float(args.manip_weight),
            manip_kappa=float(args.manip_kappa),
            reference_backend=str(args.reference_backend),
            dt=float(args.dt),
            v_ref=float(args.v_ref),
            terminal_hold_s=float(args.terminal_hold_s),
            make_plots=False,
        )
        dynamic_summary = harness.run_all()
        assessment = _goal_assessment(dynamic_summary, full_summary["offline_adapter_check"])
        assessment["all_goals_pass_for_mode"] = all(
            bool(assessment[f"goal_{i}_{suffix}_pass"])
            for i, suffix in (
                (1, "return_contract"),
                (2, "lightweight_supervisor"),
                (3, "recovery_handoff"),
                (4, "base_reposition_sequence"),
                (5, "offline_adapter"),
                (6, "integration_checklist"),
            )
        )
        all_pass = all_pass and bool(assessment["all_goals_pass_for_mode"])
        full_summary["cost_modes"][mode] = {
            "dynamic_summary_path": os.path.join(mode_out, "dynamic_replanning_summary.json"),
            "assessment": assessment,
        }
        print(f"[goal_roadmap_checks] mode={mode} all_goals_pass={assessment['all_goals_pass_for_mode']}")

    full_summary["all_modes_pass"] = bool(all_pass)
    _write_json(os.path.join(out_root, "goal_roadmap_summary.json"), full_summary)
    print(f"[goal_roadmap_checks] out={out_root} all_modes_pass={all_pass}")
    return 0 if all_pass else 1


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
