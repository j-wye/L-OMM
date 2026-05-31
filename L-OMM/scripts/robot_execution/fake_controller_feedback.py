"""Test-only scripted fake controller implementing ControllerRuntimeInterface.

It plays a commanded 7-joint trajectory back as the "actual" feedback and can
inject each abort-class fault deterministically, so every runner/monitor branch is
testable without any hardware, ROS, or real action client.  Sends are routed
through a fake-mode ``SafeTrajectoryClient`` + ``FakeFollowJointTrajectoryServer``;
no real action client is ever created.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .fake_action_server import FakeFollowJointTrajectoryServer
from .physical_dry_run_runner import (
    ControllerFeedbackFrame,
    ControllerJointState,
)
from .ros2_trajectory_client import SafeTrajectoryClient
from .safety_gate import ExecutionSafetyGate

_ACTIVE_PERTURB_JOINT = "arm_joint_5"
_FINGER_JOINT = "right_finger_bottom_joint"

# fault kinds
F_NONE = "none"
F_TRACKING_ERROR = "tracking_error"
F_FEEDBACK_TIMEOUT = "feedback_timeout"
F_REJECT = "reject"
F_ABORT = "abort"
F_NONFINITE = "nonfinite"
F_FINGER_CLOSE = "finger_close"
F_BASE_COMMAND = "base_command"
F_OPERATOR_ABORT = "operator_abort"


@dataclass(frozen=True)
class ScriptedFaults:
    kind: str = F_NONE
    at_index: int = 1
    tracking_error_rad: float = 0.10      # > 0.05 threshold
    finger_close_value: float = 0.5
    timeout_gap_s: float = 1.0            # > 0.5 threshold
    base_endpoint: str = "/cmd_vel"


class ScriptedFakeController:
    """Deterministic fake controller + feedback playback (no hardware/ROS)."""

    def __init__(
        self,
        *,
        safety_gate: ExecutionSafetyGate,
        command,
        start_joint_names: Optional[Sequence[str]] = None,
        start_positions: Optional[Sequence[float] | np.ndarray] = None,
        faults: Optional[ScriptedFaults] = None,
        finger_open: float = 0.0,
    ) -> None:
        self.command = command
        self.joint_names: Tuple[str, ...] = tuple(str(n) for n in command.joint_names)
        self.faults = faults if faults is not None else ScriptedFaults()
        self.finger_open = float(finger_open)
        self._fake_server = FakeFollowJointTrajectoryServer()
        self.client = SafeTrajectoryClient(safety_gate, fake_server=self._fake_server)
        # reported current posture (defaults to the first commanded point).
        first = np.asarray(command.points[0].positions, dtype=np.float64)
        self._start_positions = (
            first.copy() if start_positions is None else np.asarray(start_positions, dtype=np.float64).reshape(-1)
        )
        self._start_names: Tuple[str, ...] = (
            self.joint_names if start_joint_names is None else tuple(str(n) for n in start_joint_names)
        )

    # -- ControllerRuntimeInterface -------------------------------------------

    def current_joint_state(self) -> ControllerJointState:
        return ControllerJointState(
            joint_names=self._start_names,
            positions=self._start_positions.copy(),
            stamp_s=0.0,
            finite=bool(np.all(np.isfinite(self._start_positions))),
        )

    def send_goal(self, command) -> bool:
        # routed through the fake-mode SafeTrajectoryClient (records the goal).
        self.client.send(command)
        return self.faults.kind != F_REJECT

    def cancel(self) -> bool:
        return bool(self.client.cancel())

    def real_action_client_created(self) -> bool:
        return bool(self.client.as_dict().get("real_action_client_created", False))

    def feedback_frames(self) -> List[ControllerFeedbackFrame]:
        points = list(self.command.points)
        n = len(points)
        finger_idx = self.joint_names.index(_FINGER_JOINT) if _FINGER_JOINT in self.joint_names else None
        perturb_idx = (
            self.joint_names.index(_ACTIVE_PERTURB_JOINT) if _ACTIVE_PERTURB_JOINT in self.joint_names else 0
        )
        fk = self.faults.kind
        at = int(self.faults.at_index)
        frames: List[ControllerFeedbackFrame] = []
        last_stamp = 0.0
        for i, p in enumerate(points):
            actual = np.asarray(p.positions, dtype=np.float64).copy()
            stamp = float(p.time_from_start_s)
            state = "succeeded" if i == n - 1 else "executing"
            operator_abort = False
            endpoint = ""
            if i == at:
                if fk == F_TRACKING_ERROR:
                    actual[perturb_idx] += float(self.faults.tracking_error_rad)
                elif fk == F_NONFINITE:
                    actual[perturb_idx] = np.nan
                elif fk == F_FINGER_CLOSE and finger_idx is not None:
                    actual[finger_idx] = float(self.faults.finger_close_value)
                elif fk == F_FEEDBACK_TIMEOUT:
                    stamp = last_stamp + float(self.faults.timeout_gap_s)
                elif fk == F_ABORT:
                    state = "aborted"
                elif fk == F_BASE_COMMAND:
                    endpoint = str(self.faults.base_endpoint)
                elif fk == F_OPERATOR_ABORT:
                    operator_abort = True
            frames.append(
                ControllerFeedbackFrame(
                    stamp_s=stamp,
                    joint_names=self.joint_names,
                    actual_positions=actual,
                    result_state=state,
                    operator_abort=operator_abort,
                    unexpected_endpoint=endpoint,
                )
            )
            last_stamp = stamp
        return frames

    # -- test helpers ----------------------------------------------------------

    @property
    def fake_goal_count(self) -> int:
        return int(self._fake_server.goal_count)

    @property
    def fake_cancel_count(self) -> int:
        return int(self._fake_server.cancel_count)
