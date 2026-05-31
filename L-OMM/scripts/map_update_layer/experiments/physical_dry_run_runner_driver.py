#!/usr/bin/env python3
"""PHYSICAL_DRY_RUN_RUNNER driver — real-map CHECK-ONLY evidence run (no send).

Builds the contract-conforming short leading segment from the real baked-map
no-command candidate, assembles a PhysicalDryRunPlan, and runs the
PhysicalDryRunRunner in its DEFAULT check-only mode (closed authorization). It
demonstrates the full runner pipeline up to the human-gated send boundary,
without sending anything, without a real action client, and with honest
header-only feedback/tracking CSVs.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys
from typing import Any, Dict, Optional


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))


from map_update_layer.experiments.pre_physical_preflight_runner import (  # noqa: E402
    _acquire_real_accepted_candidate,
)
from robot_execution.fake_controller_feedback import ScriptedFakeController  # noqa: E402
from robot_execution.joint_mapping import ReducedJointMapper  # noqa: E402
from robot_execution.physical_dry_run_runner import (  # noqa: E402
    CHECK_ONLY_READY_NO_SEND,
    NOT_APPLICABLE_UNTIL_SEND,
    PhysicalDryRunAuthorization,
    PhysicalDryRunPlan,
    PhysicalDryRunRunner,
)
from robot_execution.pre_physical_preflight import PrePhysicalTrajectoryPreflight  # noqa: E402
from robot_execution.safety_gate import ExecutionSafetyConfig, ExecutionSafetyGate  # noqa: E402
from robot_execution.trajectory_builder import ReducedTrajectoryBuilder  # noqa: E402


def build_real_conforming_plan(out_root: Path, map_path: Optional[str] = None) -> PhysicalDryRunPlan:
    handle, candidate, provenance = _acquire_real_accepted_candidate(out_root, map_path)
    builder = ReducedTrajectoryBuilder(ReducedJointMapper())
    contract = builder.mapper.contract
    preflight = PrePhysicalTrajectoryPreflight()
    seg = preflight.conforming_leading_segment(candidate)
    if seg.candidate is None:
        raise RuntimeError(f"no contract-conforming leading segment: {seg.reason}")
    command = builder.from_active_trajectory(seg.candidate.q_active_trajectory, dt_s=seg.candidate.dt_s)
    record = preflight.evaluate(command, seg.candidate, contract=contract)
    control_dir = str(provenance.get("control_artifact_dir", ""))
    return PhysicalDryRunPlan(
        candidate=seg.candidate,
        command=command,
        preflight_record=record,
        controller_contract=contract,
        planned_start_joint_names=tuple(command.joint_names),
        planned_start_positions=command.points[0].positions.copy(),
        candidate_id=f"{handle.tag}@{control_dir}",
        control_artifact_dir=control_dir,
    )


def run_physical_dry_run_check_only(
    *, out: str = "path/physical_dry_run_check_only", map_path: Optional[str] = None
) -> Dict[str, Any]:
    out_root = Path(out)
    out_root.mkdir(parents=True, exist_ok=True)
    plan = build_real_conforming_plan(out_root, map_path)

    # fake_action gate so the controller can construct its fake client; the runner's
    # closed authorization (default) keeps the run check-only regardless.
    gate = ExecutionSafetyGate(ExecutionSafetyConfig(mode="fake_action"))
    controller = ScriptedFakeController(safety_gate=gate, command=plan.command)
    authorization = PhysicalDryRunAuthorization()  # closed: no token, no approval, no allow flag
    runner = PhysicalDryRunRunner()
    log_dir = out_root / "dry_run_logs"
    result = runner.run(
        plan, authorization=authorization, controller=controller, safety_gate=gate, out_dir=log_dir
    )

    feedback_csv = log_dir / "feedback_joint_trajectory.csv"
    feedback_lines = feedback_csv.read_text(encoding="utf-8").splitlines() if feedback_csv.exists() else []
    invariants = {
        "readiness_is_check_only": result.runner_readiness_verdict == CHECK_ONLY_READY_NO_SEND,
        "contract_not_applicable_until_send": result.contract_verdict == NOT_APPLICABLE_UNTIL_SEND,
        "no_send": (not result.goal_sent) and controller.fake_goal_count == 0,
        "no_real_action_client": not result.real_action_client_created,
        "feedback_source_none": result.feedback_source == "none",
        "physical_false": result.physical is False,
        "feedback_csv_header_only": len(feedback_lines) == 1,
        "all_csvs_present": all(
            (log_dir / n).exists()
            for n in (
                "commanded_joint_trajectory.csv", "feedback_joint_trajectory.csv",
                "tracking_error.csv", "controller_events.csv", "safety_events.csv",
            )
        ),
    }
    invariants["all_pass"] = all(invariants.values())
    status = "GOAL_STATUS: COMPLETE" if invariants["all_pass"] else "GOAL_STATUS: IN_PROGRESS"
    report = {
        "status": status,
        "mode": "real_map_physical_dry_run_check_only",
        "candidate_id": plan.candidate_id,
        "control_artifact_dir": plan.control_artifact_dir,
        "result": result.as_dict(),
        "invariants": invariants,
        "log_dir": str(log_dir),
        "manual_remaining": [
            "operator clears workspace + pre-tests e-stop (contract 2/3)",
            "operator connects the real ControllerRuntimeInterface ROS adapter",
            "operator provides the armed token + single-send approval (contract 5)",
            "operator triggers the single send + supervises live (contract 6)",
            "operator writes the FINAL PHYSICAL verdict from preserved real-run artifacts (contract 9)",
        ],
    }
    (out_root / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    return report


if __name__ == "__main__":  # pragma: no cover
    res = run_physical_dry_run_check_only()
    print(res["status"])
    print(json.dumps(res["invariants"], indent=2, ensure_ascii=False))
