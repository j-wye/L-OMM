from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))


JOINTS = (
    "arm_joint_1",
    "arm_joint_2",
    "arm_joint_3",
    "arm_joint_4",
    "arm_joint_5",
    "arm_joint_6",
    "right_finger_bottom_joint",
)


class RobotExecutionSafetyGateTest(unittest.TestCase):
    def _contract(self):
        from robot_execution.joint_mapping import ControllerContract

        return ControllerContract(
            joint_names=JOINTS,
            command_interfaces=("position",),
            state_interfaces=("position", "velocity"),
            allow_partial_joints_goal=False,
            state_publish_rate_hz=100.0,
            update_rate_hz=100.0,
        )

    def test_safety_gate_blocks_real_hardware_by_default(self) -> None:
        from robot_execution.safety_gate import ExecutionSafetyGate, SafetyViolation

        gate = ExecutionSafetyGate()
        with self.assertRaises(SafetyViolation):
            gate.assert_can_create_action_client("/arm_controller/follow_joint_trajectory")
        with self.assertRaises(SafetyViolation):
            gate.assert_can_send_goal("/arm_controller/follow_joint_trajectory")
        self.assertEqual(gate.as_dict()["mode"], "dry_run")
        self.assertFalse(gate.as_dict()["armed"])

    def test_safety_gate_allows_fake_action_server_only_in_fake_mode(self) -> None:
        from robot_execution.fake_action_server import FakeFollowJointTrajectoryServer
        from robot_execution.ros2_trajectory_client import SafeTrajectoryClient
        from robot_execution.safety_gate import ExecutionSafetyConfig, ExecutionSafetyGate

        gate = ExecutionSafetyGate(ExecutionSafetyConfig(mode="fake_action"))
        server = FakeFollowJointTrajectoryServer()
        client = SafeTrajectoryClient(gate, fake_server=server)
        command = self._trajectory_command()

        status = client.send(command)

        self.assertEqual(status.state, "accepted")
        self.assertEqual(server.goal_count, 1)
        self.assertEqual(server.cancel_count, 0)
        self.assertFalse(client.as_dict()["real_action_client_created"])

    def test_real_action_send_is_unreachable_without_armed_token(self) -> None:
        from robot_execution.ros2_trajectory_client import SafeTrajectoryClient
        from robot_execution.safety_gate import ExecutionSafetyConfig, ExecutionSafetyGate, SafetyViolation

        gate = ExecutionSafetyGate(
            ExecutionSafetyConfig(
                mode="armed",
                armed=True,
                armed_token="wrong-token",
                allow_real_action_client=True,
            )
        )
        client = SafeTrajectoryClient(gate, use_fake_server=False)

        with self.assertRaises(SafetyViolation):
            client.send(self._trajectory_command())

    def test_reduced_q_maps_to_exact_full_controller_joint_order(self) -> None:
        from constants import Q1_FIXED, Q4_FIXED, Q6_FIXED
        from robot_execution.joint_mapping import ReducedJointMapper

        mapper = ReducedJointMapper(self._contract())
        full = mapper.full_from_active([0.2, -0.3, 0.4])

        self.assertEqual(tuple(full.joint_names), JOINTS)
        np.testing.assert_allclose(
            full.positions,
            np.array([Q1_FIXED, 0.2, -0.3, Q4_FIXED, 0.4, Q6_FIXED, 0.0]),
        )

    def test_partial_goal_rejected_when_controller_disallows_partial_goals(self) -> None:
        from robot_execution.joint_mapping import JointMappingError, ReducedJointMapper

        mapper = ReducedJointMapper(self._contract())
        with self.assertRaises(JointMappingError):
            mapper.validate_goal_joint_names(("arm_joint_2", "arm_joint_3", "arm_joint_5"))

    def test_trajectory_builder_rejects_invalid_trajectories(self) -> None:
        from robot_execution.joint_mapping import ReducedJointMapper
        from robot_execution.trajectory_builder import ReducedTrajectoryBuilder, TrajectoryLimits, TrajectoryValidationError

        mapper = ReducedJointMapper(self._contract())
        builder = ReducedTrajectoryBuilder(mapper, TrajectoryLimits(max_velocity_rad_s=0.5))

        with self.assertRaises(TrajectoryValidationError):
            builder.from_active_trajectory(np.zeros((0, 3)), dt_s=1.0)
        with self.assertRaises(TrajectoryValidationError):
            builder.from_active_trajectory([[0.0, 0.0, 0.0], [np.nan, 0.0, 0.0]], dt_s=1.0)
        with self.assertRaises(TrajectoryValidationError):
            builder.from_active_trajectory([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]], timestamps_s=[0.0, 0.0])
        with self.assertRaises(TrajectoryValidationError):
            builder.from_active_trajectory([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dt_s=1.0)

    def test_progress_estimator_projects_joint_state_and_rejects_stale_state(self) -> None:
        from robot_execution.joint_mapping import ReducedJointMapper
        from robot_execution.progress_estimator import ProgressEstimator, ProgressEstimatorError

        mapper = ReducedJointMapper(self._contract())
        estimator = ProgressEstimator(mapper, max_state_age_s=0.5)
        q_traj = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        s_table = np.array([0.0, 1.0, 2.0])

        estimate = estimator.estimate_from_joint_state(
            joint_names=JOINTS,
            positions=(0.0, 1.05, 0.0, 0.0, 0.0, 0.0, 0.0),
            stamp_s=10.0,
            now_s=10.2,
            s_table=s_table,
            q_active_trajectory=q_traj,
        )
        self.assertAlmostEqual(estimate.progress_s, 1.0)
        self.assertFalse(estimate.stale)

        with self.assertRaises(ProgressEstimatorError):
            estimator.estimate_from_joint_state(
                joint_names=JOINTS,
                positions=(0.0, 1.05, 0.0, 0.0, 0.0, 0.0, 0.0),
                stamp_s=10.0,
                now_s=11.0,
                s_table=s_table,
                q_active_trajectory=q_traj,
            )

    def test_no_command_scanner_for_robot_execution_tests(self) -> None:
        test_dir = Path(__file__).resolve().parent
        texts = []
        for path in test_dir.glob("test_robot_execution*.py"):
            texts.append(path.read_text(encoding="utf-8"))
        for path in test_dir.glob("test_execution_harness*.py"):
            texts.append(path.read_text(encoding="utf-8"))
        combined = "\n".join(texts)
        forbidden = (
            "ros2 action " + "send_goal",
            "ros2 topic " + "pub",
            "create_" + "publisher(",
            "Action" + "Client(",
            "/" + "cmd_vel",
        )
        for token in forbidden:
            self.assertNotIn(token, combined)

    def _trajectory_command(self):
        from robot_execution.joint_mapping import ReducedJointMapper
        from robot_execution.trajectory_builder import ReducedTrajectoryBuilder

        mapper = ReducedJointMapper(self._contract())
        builder = ReducedTrajectoryBuilder(mapper)
        return builder.from_active_trajectory(
            [[0.1, 0.2, 0.3], [0.12, 0.21, 0.31]],
            dt_s=1.0,
            metadata={"test": "fake_action_only"},
        )


if __name__ == "__main__":
    unittest.main()
