from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
import uuid

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))


class ExecutionHarnessIntegrationClosureTest(unittest.TestCase):
    def _tmp_dir(self) -> Path:
        path = Path(tempfile.gettempdir()) / "ag_exec_harness_closure" / uuid.uuid4().hex[:12]
        path.mkdir(parents=True, exist_ok=True)
        return path

    def _run_control_integration(self) -> Path:
        from map_update_layer.experiments.control_passive_map_update_integration import run_integration

        out = self._tmp_dir()
        report = run_integration(out=str(out), mode="synthetic", run_jetson=False)
        self.assertTrue(report["initial_candidate_accepted"])
        self.assertTrue(report["replan_triggered"])
        return out

    def test_control_module_artifacts_bridge_to_execution_candidate_schema(self) -> None:
        from robot_execution.control_bridge import execution_candidate_from_control_artifacts

        out = self._run_control_integration()
        candidate = execution_candidate_from_control_artifacts(
            out / "stage_b_initial_control_candidate" / "initial_control"
        )

        self.assertTrue(candidate.accepted)
        self.assertEqual(candidate.q_active_trajectory.shape[1], 3)
        self.assertEqual(candidate.s_table.shape[0], candidate.q_active_trajectory.shape[0])
        self.assertEqual(candidate.xz_table.shape[0], candidate.q_active_trajectory.shape[0])
        self.assertGreater(candidate.q_active_trajectory.shape[0], 100)
        self.assertAlmostEqual(float(candidate.dt_s), 0.01)
        self.assertEqual(candidate.metadata["active_joint_order"], ["q2", "q3", "q5"])
        self.assertEqual(candidate.metadata["reference_support"], "episode_s_interpolated_reference_xz")

    def test_full_stack_fake_action_consumes_control_candidate_and_replan_event(self) -> None:
        from robot_execution.control_bridge import execution_candidate_from_control_artifacts
        from robot_execution.execution_supervisor import MovingArmExecutionSupervisor
        from robot_execution.fake_action_server import FakeFollowJointTrajectoryServer
        from robot_execution.joint_mapping import ReducedJointMapper
        from robot_execution.ros2_trajectory_client import SafeTrajectoryClient
        from robot_execution.safety_gate import ExecutionSafetyConfig, ExecutionSafetyGate
        from robot_execution.trajectory_builder import ReducedTrajectoryBuilder

        out = self._run_control_integration()
        candidate = execution_candidate_from_control_artifacts(
            out / "stage_b_initial_control_candidate" / "initial_control",
            execution_dt_s=0.1,
        )
        report_data = json.loads(
            (out / "stage_c_dynamic_blocked_path_trigger" / "blocked_path_report.json").read_text(encoding="utf-8")
        )
        blocked = np.zeros((8, 8), dtype=bool)
        blocked[3, 3] = True
        snapshot = SimpleNamespace(blocked_mask=blocked, final_active_mask=~blocked, stats={"sequence_id": 2})
        report = SimpleNamespace(
            event_class=report_data["event_class"],
            replanning_triggered=bool(report_data["replanning_triggered"]),
            reason=report_data["reason"],
            current_s_m=float(report_data["current_s_m"]),
        )

        mapper = ReducedJointMapper()
        builder = ReducedTrajectoryBuilder(mapper)
        gate = ExecutionSafetyGate(ExecutionSafetyConfig(mode="fake_action"))
        server = FakeFollowJointTrajectoryServer()
        client = SafeTrajectoryClient(gate, fake_server=server)
        supervisor = MovingArmExecutionSupervisor(trajectory_builder=builder, trajectory_client=client)

        start = supervisor.start(candidate, snapshot)
        event_result = supervisor.handle_event(report, snapshot=snapshot, candidate=None)

        self.assertEqual(start.action, "started")
        self.assertEqual(event_result.action, "replan_required")
        self.assertEqual(server.goal_count, 1)
        self.assertEqual(server.cancel_count, 1)
        self.assertFalse(client.as_dict()["real_action_client_created"])
        self.assertEqual(tuple(supervisor.current_command.joint_names), mapper.contract.joint_names)
        self.assertEqual(len(supervisor.current_command.points[0].positions), 7)
        self.assertAlmostEqual(float(supervisor.current_command.points[-1].positions[-1]), 0.0)


if __name__ == "__main__":
    unittest.main()
