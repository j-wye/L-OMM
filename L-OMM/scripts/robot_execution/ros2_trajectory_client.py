from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict

from .fake_action_server import FakeFollowJointTrajectoryServer
from .safety_gate import ExecutionSafetyGate


@dataclass(frozen=True)
class TrajectoryClientStatus:
    state: str
    reason: str = ""
    goal_id: int | None = None

    def as_dict(self) -> Dict[str, object]:
        return {
            "state": self.state,
            "reason": self.reason,
            "goal_id": self.goal_id,
        }


class SafeTrajectoryClient:
    """Safety-gated FollowJointTrajectory boundary.

    Fake mode never imports or creates ROS2 action objects.  Real action-client
    creation and goal sending are separate gated operations.
    """

    def __init__(
        self,
        safety_gate: ExecutionSafetyGate,
        *,
        action_name: str = "/arm_controller/follow_joint_trajectory",
        fake_server: FakeFollowJointTrajectoryServer | None = None,
        use_fake_server: bool | None = None,
        real_sender: Any | None = None,
    ) -> None:
        self.safety_gate = safety_gate
        self.action_name = str(action_name)
        self.fake_server = fake_server
        self.use_fake_server = bool(fake_server is not None) if use_fake_server is None else bool(use_fake_server)
        self.real_sender = real_sender
        self._real_action_client: Any | None = None
        self._real_action_client_created = False
        if self.use_fake_server:
            self.safety_gate.assert_can_create_action_client(self.action_name, fake=True)
            if self.fake_server is None:
                self.fake_server = FakeFollowJointTrajectoryServer()

    def create_real_action_client(self, *, node: Any = None, action_client_factory: Any = None) -> Any:
        self.safety_gate.assert_can_create_action_client(self.action_name, fake=False)
        if action_client_factory is not None:
            self._real_action_client = action_client_factory(node, self.action_name)
            self._real_action_client_created = True
            return self._real_action_client
        try:
            from rclpy.action import ActionClient
            from control_msgs.action import FollowJointTrajectory
        except Exception as exc:
            raise RuntimeError("ROS2 action dependencies are unavailable in this environment") from exc
        if node is None:
            raise RuntimeError("a ROS2 node is required to create a real action client")
        self._real_action_client = ActionClient(node, FollowJointTrajectory, self.action_name)
        self._real_action_client_created = True
        return self._real_action_client

    def send(self, command) -> TrajectoryClientStatus:
        self.safety_gate.assert_trajectory_safe(command)
        if self.use_fake_server:
            self.safety_gate.assert_can_send_goal(self.action_name, fake=True)
            record = self.fake_server.send_goal(command)  # type: ignore[union-attr]
            return TrajectoryClientStatus(state="accepted", reason="fake_action_goal_recorded", goal_id=record.goal_id)

        self.safety_gate.assert_can_send_goal(self.action_name, fake=False)
        if self.real_sender is not None:
            result = self.real_sender(command)
            return TrajectoryClientStatus(state="sent", reason="real_sender_invoked", goal_id=getattr(result, "goal_id", None))
        if self._real_action_client is None:
            raise RuntimeError("real action client has not been created")
        goal_msg = self._to_ros_goal(command)
        future = self._real_action_client.send_goal_async(goal_msg)
        return TrajectoryClientStatus(state="sent", reason="real_action_client_send_goal_async_called", goal_id=None)

    def cancel(self) -> TrajectoryClientStatus:
        if self.use_fake_server:
            cancelled = self.fake_server.cancel() if self.fake_server is not None else False
            return TrajectoryClientStatus(
                state="cancelled" if cancelled else "no_active_goal",
                reason="fake_action_cancel",
                goal_id=None,
            )
        if self._real_action_client is None:
            return TrajectoryClientStatus(state="no_active_goal", reason="no_real_action_client")
        if not hasattr(self._real_action_client, "cancel_goal_async"):
            return TrajectoryClientStatus(state="cancel_unavailable", reason="real client has no cancel_goal_async")
        self._real_action_client.cancel_goal_async()
        return TrajectoryClientStatus(state="cancel_requested", reason="real_action_cancel_goal_async_called")

    def as_dict(self) -> Dict[str, object]:
        return {
            "action_name": self.action_name,
            "fake_mode": bool(self.use_fake_server),
            "real_action_client_created": bool(self._real_action_client_created),
            "safety_gate": self.safety_gate.as_dict(),
        }

    @staticmethod
    def _to_ros_goal(command):
        try:
            from builtin_interfaces.msg import Duration
            from control_msgs.action import FollowJointTrajectory
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        except Exception as exc:
            raise RuntimeError("ROS2 trajectory message dependencies are unavailable") from exc

        trajectory = JointTrajectory()
        trajectory.joint_names = list(command.joint_names)
        for point in command.points:
            msg = JointTrajectoryPoint()
            msg.positions = [float(v) for v in point.positions.tolist()]
            if point.velocities is not None:
                msg.velocities = [float(v) for v in point.velocities.tolist()]
            sec = int(point.time_from_start_s)
            nanosec = int(round((float(point.time_from_start_s) - sec) * 1.0e9))
            msg.time_from_start = Duration(sec=sec, nanosec=nanosec)
            trajectory.points.append(msg)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        return goal

