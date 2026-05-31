from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
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


@dataclass
class FakeHandle:
    shape: tuple[int, int] = (4, 4)
    tag: str = "fake_0.047y_0.5z"
    resolution_m: float = 0.01
    x0: float = 0.0
    z0: float = 0.0

    @property
    def meta(self):
        return {"target_height": 0.5}


class ReferenceRecorder:
    def __init__(self) -> None:
        self.calls: list[dict[str, object]] = []

    def capture(self, snapshot, q_act, T_base_cam, reference_corridor=None, metadata=None) -> None:
        self.calls.append(
            {
                "snapshot": snapshot,
                "q_act": np.asarray(q_act, dtype=np.float64).copy(),
                "reference_corridor": None if reference_corridor is None else np.asarray(reference_corridor).copy(),
                "metadata": dict(metadata or {}),
            }
        )

    def invalidate(self) -> None:
        self.calls.append({"invalidated": True})


class RobotExecutionSupervisorTest(unittest.TestCase):
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

    def _supervisor(self, *, with_history: bool = False, backtrack_on_recovery: bool = False):
        history = None
        from robot_execution.execution_supervisor import MovingArmExecutionSupervisor
        from robot_execution.fake_action_server import FakeFollowJointTrajectoryServer
        from robot_execution.joint_mapping import ReducedJointMapper
        from robot_execution.ros2_trajectory_client import SafeTrajectoryClient
        from robot_execution.safety_gate import ExecutionSafetyConfig, ExecutionSafetyGate
        from robot_execution.trajectory_builder import ReducedTrajectoryBuilder

        mapper = ReducedJointMapper(self._contract())
        builder = ReducedTrajectoryBuilder(mapper)
        if with_history:
            from robot_execution.execution_history import ExecutionHistoryRecorder

            history = ExecutionHistoryRecorder(mapper)
        gate = ExecutionSafetyGate(ExecutionSafetyConfig(mode="fake_action"))
        server = FakeFollowJointTrajectoryServer()
        client = SafeTrajectoryClient(gate, fake_server=server)
        recorder = ReferenceRecorder()
        supervisor = MovingArmExecutionSupervisor(
            trajectory_builder=builder,
            trajectory_client=client,
            reference_recorder=recorder,
            execution_history=history,
            backtrack_on_recovery=backtrack_on_recovery,
        )
        return supervisor, server, recorder, history

    def _snapshot(self, blocked_cell: tuple[int, int] | None = None):
        blocked = np.zeros((4, 4), dtype=bool)
        if blocked_cell is not None:
            blocked[blocked_cell] = True
        final = ~blocked
        return SimpleNamespace(
            handle=FakeHandle(),
            blocked_mask=blocked,
            final_active_mask=final,
            stats={"map_update_sequence_id": 7},
        )

    def _candidate(self, *, accepted: bool = True, q_offset: float = 0.0):
        from robot_execution.execution_supervisor import ExecutionCandidate

        q = np.array(
            [
                [0.10 + q_offset, 0.20, 0.30],
                [0.11 + q_offset, 0.21, 0.31],
                [0.12 + q_offset, 0.22, 0.32],
            ],
            dtype=np.float64,
        )
        return ExecutionCandidate(
            q_active_trajectory=q,
            s_table=np.array([0.0, 0.01, 0.02], dtype=np.float64),
            xz_table=np.array([[0.0, 0.0], [0.01, 0.0], [0.02, 0.0]], dtype=np.float64),
            dt_s=1.0,
            candidate_metrics={
                "candidate_accepted": accepted,
                "rejection_reason": "" if accepted else "UNGRASPABLE",
                "invalid_reason_code": "" if accepted else "goal_final_mask_blocked",
            },
            metadata={"source": "unit_test"},
        )

    def _report(self, event: str, *, triggered: bool = True, reason: str = "unit_test"):
        return SimpleNamespace(
            event_class=event,
            replanning_triggered=triggered,
            reason=reason,
            current_s_m=0.01,
        )

    def test_initial_acceptance_sends_fake_goal_and_captures_reference_once(self) -> None:
        supervisor, server, recorder, _ = self._supervisor()

        result = supervisor.start(self._candidate(), self._snapshot())

        self.assertEqual(result.action, "started")
        self.assertEqual(server.goal_count, 1)
        self.assertEqual(result.reference_capture_count, 1)
        self.assertEqual(len(recorder.calls), 1)
        self.assertEqual(recorder.calls[0]["q_act"].shape, (3, 3))

    def test_accepted_replan_cancels_switches_and_recaptures_reference(self) -> None:
        from map_update_layer.runtime_contract import RuntimeEventClass

        supervisor, server, recorder, _ = self._supervisor()
        supervisor.start(self._candidate(), self._snapshot())

        result = supervisor.handle_event(
            self._report(RuntimeEventClass.REFERENCE_BLOCKED),
            snapshot=self._snapshot((1, 1)),
            candidate=self._candidate(q_offset=0.02),
        )

        self.assertEqual(result.action, "switched_to_accepted_reference")
        self.assertEqual(server.cancel_count, 1)
        self.assertEqual(server.goal_count, 2)
        self.assertEqual(result.reference_capture_count, 2)
        self.assertEqual(len([call for call in recorder.calls if "q_act" in call]), 2)

    def test_current_pose_unsafe_cancels_and_enters_recovery_boundary(self) -> None:
        from map_update_layer.runtime_contract import RuntimeEventClass

        supervisor, server, _, _ = self._supervisor()
        supervisor.start(self._candidate(), self._snapshot())

        result = supervisor.handle_event(
            self._report(RuntimeEventClass.CURRENT_POSE_UNSAFE, reason="current_pose_intersects_blocked_mask"),
            snapshot=self._snapshot((0, 0)),
        )

        self.assertEqual(result.action, "safety_recovery_boundary")
        self.assertEqual(server.cancel_count, 1)
        self.assertTrue(result.recovery_required)
        self.assertIsNotNone(result.recovery_request)

    def test_same_blocked_mask_rejected_replan_is_not_retried_indefinitely(self) -> None:
        from map_update_layer.runtime_contract import RuntimeEventClass

        snapshot = self._snapshot((2, 2))
        supervisor, server, _, _ = self._supervisor()
        supervisor.start(self._candidate(), self._snapshot())

        first = supervisor.handle_event(
            self._report(RuntimeEventClass.GOAL_CORRIDOR_BLOCKED),
            snapshot=snapshot,
            candidate=self._candidate(accepted=False),
        )
        second = supervisor.handle_event(
            self._report(RuntimeEventClass.GOAL_CORRIDOR_BLOCKED),
            snapshot=snapshot,
            candidate=self._candidate(accepted=False),
        )

        self.assertEqual(first.action, "recovery_request")
        self.assertEqual(second.action, "same_snapshot_retry_prevented")
        self.assertTrue(second.same_snapshot_retry_prevented)
        self.assertEqual(server.cancel_count, 1)

    def test_mask_changed_noncritical_continues_current_reference_without_replan(self) -> None:
        from map_update_layer.runtime_contract import RuntimeEventClass

        supervisor, server, _, _ = self._supervisor()
        supervisor.start(self._candidate(), self._snapshot())

        result = supervisor.handle_event(
            self._report(RuntimeEventClass.MASK_CHANGED_NONCRITICAL, triggered=False),
            snapshot=self._snapshot((3, 3)),
        )

        self.assertEqual(result.action, "continue_current_reference")
        self.assertFalse(result.replanning_triggered)
        self.assertEqual(server.cancel_count, 0)
        self.assertEqual(server.goal_count, 1)

    def test_history_recorder_builds_reverse_backtrack_preserving_finger_joint(self) -> None:
        from robot_execution.execution_history import ExecutionHistoryRecorder
        from robot_execution.joint_mapping import ReducedJointMapper

        mapper = ReducedJointMapper(self._contract())
        history = ExecutionHistoryRecorder(mapper)
        history.record_active_state([0.10, 0.20, 0.30], stamp_s=1.0, finger_position=0.9)
        history.record_active_state([0.11, 0.21, 0.31], stamp_s=2.0, finger_position=0.9)
        history.record_active_state([0.12, 0.22, 0.32], stamp_s=3.0, finger_position=0.9)

        command = history.build_reverse_command()

        self.assertTrue(command.summary["backtrack"])
        self.assertEqual(command.joint_names, JOINTS)
        self.assertEqual(len(command.points), 3)
        np.testing.assert_allclose(command.points[0].positions[[1, 2, 4]], [0.12, 0.22, 0.32])
        np.testing.assert_allclose(command.points[-1].positions[[1, 2, 4]], [0.10, 0.20, 0.30])
        self.assertTrue(all(abs(point.positions[6] - 0.9) < 1.0e-12 for point in command.points))

    def test_rejected_replan_can_start_fake_backtrack_from_recorded_history(self) -> None:
        from map_update_layer.runtime_contract import RuntimeEventClass

        supervisor, server, _, history = self._supervisor(with_history=True, backtrack_on_recovery=True)
        assert history is not None
        history.record_active_state([0.10, 0.20, 0.30], stamp_s=1.0, finger_position=0.9)
        history.record_active_state([0.11, 0.21, 0.31], stamp_s=2.0, finger_position=0.9)
        history.record_active_state([0.12, 0.22, 0.32], stamp_s=3.0, finger_position=0.9)
        supervisor.start(self._candidate(), self._snapshot())

        result = supervisor.handle_event(
            self._report(RuntimeEventClass.GOAL_CORRIDOR_BLOCKED),
            snapshot=self._snapshot((2, 2)),
            candidate=self._candidate(accepted=False),
        )

        self.assertEqual(result.action, "recovery_request")
        self.assertTrue(result.backtrack_goal_sent)
        self.assertEqual(server.cancel_count, 1)
        self.assertEqual(server.goal_count, 2)
        self.assertTrue(server.goals[-1].summary["backtrack"])
        np.testing.assert_allclose(server.goals[-1].points[0].positions[[1, 2, 4]], [0.12, 0.22, 0.32])

    def test_retry_would_be_prevented_mirrors_hash_gate_without_mutation(self) -> None:
        from map_update_layer.runtime_contract import RuntimeEventClass

        supervisor, server, _, _ = self._supervisor()
        supervisor.start(self._candidate(), self._snapshot())

        blocked_snapshot = self._snapshot((2, 2))
        other_snapshot = self._snapshot((1, 1))

        # before any recovery hash is set, the precheck is False.
        self.assertFalse(supervisor.retry_would_be_prevented(blocked_snapshot))

        # a rejected-candidate recovery sets _last_recovery_blocked_hash for this mask.
        first = supervisor.handle_event(
            self._report(RuntimeEventClass.GOAL_CORRIDOR_BLOCKED),
            snapshot=blocked_snapshot,
            candidate=self._candidate(accepted=False),
        )
        self.assertEqual(first.action, "recovery_request")

        # capture supervisor state + fake-server counters before the read-only precheck.
        hash_before = supervisor._last_recovery_blocked_hash
        capture_before = supervisor.reference_capture_count
        candidate_before = supervisor.current_candidate
        cancel_before = server.cancel_count
        goal_before = server.goal_count

        # same blocked mask -> True (even a fresh array with the same content); different -> False.
        self.assertTrue(supervisor.retry_would_be_prevented(blocked_snapshot))
        self.assertTrue(supervisor.retry_would_be_prevented(self._snapshot((2, 2))))
        self.assertFalse(supervisor.retry_would_be_prevented(other_snapshot))

        # the precheck mutates NO supervisor state and sends NO fake command.
        self.assertEqual(supervisor._last_recovery_blocked_hash, hash_before)
        self.assertEqual(supervisor.reference_capture_count, capture_before)
        self.assertIs(supervisor.current_candidate, candidate_before)
        self.assertEqual(server.cancel_count, cancel_before)
        self.assertEqual(server.goal_count, goal_before)

        # it agrees with the actual handle_event hash gate: candidate=None still
        # short-circuits to same_snapshot_retry_prevented for the same blocked mask.
        second = supervisor.handle_event(
            self._report(RuntimeEventClass.GOAL_CORRIDOR_BLOCKED),
            snapshot=self._snapshot((2, 2)),
            candidate=None,
        )
        self.assertEqual(second.action, "same_snapshot_retry_prevented")
        self.assertTrue(second.same_snapshot_retry_prevented)

    def test_backtrack_failure_does_not_block_recovery_request(self) -> None:
        from map_update_layer.runtime_contract import RuntimeEventClass

        supervisor, server, _, _ = self._supervisor(with_history=True, backtrack_on_recovery=True)
        supervisor.start(self._candidate(), self._snapshot())

        result = supervisor.handle_event(
            self._report(RuntimeEventClass.GOAL_CORRIDOR_BLOCKED),
            snapshot=self._snapshot((2, 2)),
            candidate=self._candidate(accepted=False),
        )

        self.assertEqual(result.action, "recovery_request")
        self.assertTrue(result.recovery_required)
        self.assertFalse(result.backtrack_goal_sent)
        self.assertIn("backtrack_error", result.command_status)
        self.assertEqual(server.goal_count, 1)


if __name__ == "__main__":
    unittest.main()
