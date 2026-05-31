#!/usr/bin/env python3
"""PRE_PHYSICAL_TRAJECTORY_PREFLIGHT runner (offline, no-command).

Pulls the real baked-map no-command accepted candidate (retimed dt=0.1), builds
the 7-joint command, and runs the safety-contract trajectory preflight:

  - the FULL reference is (correctly) too large for a first object-free dry-run
    -> FAIL_PRECHECK on EE displacement;
  - a contract-conforming SHORT leading segment (<= EE cap) is derived and shown
    to PASS the trajectory preflight (eligible, subject to the human-only fields);
  - the safety gate is proven offline to block any real send without the token.

It NEVER moves the robot, arms the system, creates a real action client, sends a
goal, or exports the armed token. Artifacts only.
"""
from __future__ import annotations

import csv
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


from map_update_layer.experiments.no_command_orchestrator import (  # noqa: E402
    DEFAULT_MU_MIN,
    EXECUTION_DT_S,
    ControlModule,
    FrameSpec,
    NoCommandOrchestrator,
    build_real_handle,
    frontoparallel_camera,
)
from robot_execution.joint_mapping import ReducedJointMapper  # noqa: E402
from robot_execution.pre_physical_preflight import (  # noqa: E402
    PrePhysicalTrajectoryPreflight,
    default_no_token_gate,
)
from robot_execution.trajectory_builder import ReducedTrajectoryBuilder  # noqa: E402


def _acquire_real_accepted_candidate(out_root: Path, map_path: Optional[str]):
    """Reuse the prior closure path to obtain the accepted dt=0.1 candidate."""

    handle = build_real_handle(map_path)
    intr, T = frontoparallel_camera(handle)
    control = ControlModule(output_root=str(out_root / "control_root"))
    orch = NoCommandOrchestrator(
        handle,
        mu_min=DEFAULT_MU_MIN,
        use_path_collision_monitor=True,
        intrinsics=intr,
        T_base_cam=T,
        control_module=control,
        control_out_root=str(out_root / "control_probe"),
    )
    raw0 = orch._build_snapshot(FrameSpec(label="preflight_probe_clear"), 0)
    sticky0 = orch._apply_sticky(raw0)
    candidate, _ref, provenance = orch._acquire_initial_reference(sticky0, 0)
    return handle, candidate, provenance


def _write_commanded_trajectory_csv(path: Path, command) -> None:
    """log-schema §6 commanded_joint_trajectory.csv for the conforming segment."""

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["sample_index", "time_from_start_s", "joint_1", "joint_2", "joint_3",
             "joint_4", "joint_5", "joint_6", "finger"]
        )
        for i, point in enumerate(command.points):
            pos = [float(v) for v in point.positions.tolist()]
            writer.writerow([i, float(point.time_from_start_s), *pos])


def run_pre_physical_preflight(
    *, out: str = "path/pre_physical_preflight", map_path: Optional[str] = None
) -> Dict[str, Any]:
    out_root = Path(out)
    out_root.mkdir(parents=True, exist_ok=True)

    handle, candidate, provenance = _acquire_real_accepted_candidate(out_root, map_path)
    builder = ReducedTrajectoryBuilder(ReducedJointMapper())
    contract = builder.mapper.contract
    preflight = PrePhysicalTrajectoryPreflight()
    control_artifact_dir = str(provenance.get("control_artifact_dir", ""))
    candidate_id = f"{handle.tag}@{control_artifact_dir}"

    # 1) full reference: expected FAIL_PRECHECK (too large for a first object-free test).
    command_full = builder.from_active_trajectory(candidate.q_active_trajectory, dt_s=candidate.dt_s)
    full_record = preflight.evaluate(command_full, candidate, contract=contract)

    # 2) contract-conforming short leading segment: expected PASS-eligible.
    seg = preflight.conforming_leading_segment(candidate)
    conforming_record = None
    conforming_command = None
    if seg.candidate is not None:
        conforming_command = builder.from_active_trajectory(
            seg.candidate.q_active_trajectory, dt_s=seg.candidate.dt_s
        )
        conforming_record = preflight.evaluate(conforming_command, seg.candidate, contract=contract)

    # 3) offline arming-gate proof (no real send / no token).
    gate_proof = preflight.assert_no_command_offline(default_no_token_gate())

    # Artifacts: log-schema sections + the conforming commanded trajectory CSV.
    log_sections = (
        preflight.emit_log_schema_sections(
            conforming_record, contract=contract, candidate_id=candidate_id,
            control_artifact_dir=control_artifact_dir,
        )
        if conforming_record is not None else None
    )
    if conforming_command is not None:
        _write_commanded_trajectory_csv(out_root / "commanded_joint_trajectory.csv", conforming_command)

    invariants = {
        "full_is_fail_precheck": full_record.verdict_eligibility == "FAIL_PRECHECK",
        "full_failed_on_ee_displacement": not bool(full_record.checks.get("ee_displacement_within_cap", True)),
        "conforming_segment_exists": bool(seg.candidate is not None),
        "conforming_is_pass_eligible": bool(conforming_record is not None and conforming_record.passed),
        "real_send_blocked_without_token": bool(gate_proof["real_send_blocked_without_token"]),
        "armed_token_not_hardcoded_as_armed": bool(gate_proof["armed_token_not_hardcoded_as_armed"]),
        "allow_real_action_client_false": bool(gate_proof["allow_real_action_client_false"]),
        "no_real_action_client": True,  # this runner never creates one
        "execution_dt_is_hardware_cadence": bool(
            conforming_record is not None and abs(conforming_record.execution_dt_s - EXECUTION_DT_S) < 1e-9
        ),
    }
    invariants["all_pass"] = all(invariants.values())
    status = "GOAL_STATUS: COMPLETE" if invariants["all_pass"] else "GOAL_STATUS: IN_PROGRESS"

    report = {
        "status": status,
        "mode": "offline_pre_physical_trajectory_preflight",
        "map_tag": handle.tag,
        "candidate_id": candidate_id,
        "control_artifact_dir": control_artifact_dir,
        "ee_cap_m": preflight.config.max_ee_displacement_m,
        "full_record": full_record.as_dict(),
        "conforming_segment": {
            "n_points": int(seg.n_points),
            "path_length_m": float(seg.path_length_m),
            "reason": seg.reason,
            "record": None if conforming_record is None else conforming_record.as_dict(),
        },
        "gate_proof": gate_proof,
        "log_schema_sections": log_sections,
        "invariants": invariants,
        "manual_remaining": [
            "workspace/robot preconditions (contract 2/3)",
            "real arming with the user-provided token (contract 5)",
            "single approved send + live monitoring (contract 6)",
            "post-run log collection and final verdict signoff (contract 9)",
        ],
    }
    (out_root / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    return report


if __name__ == "__main__":  # pragma: no cover
    res = run_pre_physical_preflight()
    print(res["status"])
    print(json.dumps(res["invariants"], indent=2, ensure_ascii=False))
