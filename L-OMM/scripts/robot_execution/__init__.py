"""No-command moving-arm execution harness components.

The package is intentionally outside the passive map-update/control modules.
Default construction remains dry-run and cannot create or send real hardware
commands without an explicit armed safety gate.
"""

from .safety_gate import ExecutionSafetyConfig, ExecutionSafetyGate, SafetyViolation
from .joint_mapping import ControllerContract, FixedPosture, ReducedJointMapper
from .trajectory_builder import JointTrajectoryCommand, ReducedTrajectoryBuilder, TrajectoryLimits
from .execution_history import ExecutionHistoryError, ExecutionHistoryRecorder, ExecutionHistorySample
from .control_bridge import ControlArtifactBridgeError, execution_candidate_from_control_artifacts

__all__ = [
    "ControlArtifactBridgeError",
    "ControllerContract",
    "ExecutionHistoryError",
    "ExecutionHistoryRecorder",
    "ExecutionHistorySample",
    "ExecutionSafetyConfig",
    "ExecutionSafetyGate",
    "FixedPosture",
    "JointTrajectoryCommand",
    "ReducedJointMapper",
    "ReducedTrajectoryBuilder",
    "SafetyViolation",
    "TrajectoryLimits",
    "execution_candidate_from_control_artifacts",
]
